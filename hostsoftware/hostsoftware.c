/* $Id: hostsoftware.c,v 1.15 2014/06/06 17:38:50 simimeie Exp $
 * This is the control software for the ds1820-to-usb device that runs
 * on the USB host to which the device is connected.
 * You will need libusb and its development headers to compile this.
 * On Linux, you will usually have to run this as root to be allowed to
 * access the device, unless you configure udev accordingly.
 * This is based on powerSwitch.c, included in the reference implementation
 * from Objective Development Software GmbH for the avrusb library from the
 * same company.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <libusb-1.0/libusb.h> /* libusb-1.0, http://libusb.sourceforge.net/ */
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <sys/select.h>  /* According to POSIX.1-2001 */

int verblev = 1;
#define VERBPRINT(lev, fmt...) \
        if (verblev > lev) { \
          printf(fmt); \
          fflush(stdout); \
        }
int devicenr = 1;
int runinforeground = 0;
int restartonerror = 0;

#define USBDEV_SHARED_VENDOR    0x16C0  /* VOTI */
#define USBDEV_SHARED_PRODUCT   0x05DC  /* Obdev's free shared PID */
/* Use obdev's generic shared VID/PID pair and follow the rules outlined
 * in firmware/usbdrv/USBID-License.txt.
 */

/* These are the vendor specific SETUP commands implemented by our USB device */
#define CMD_ECHO          0  /* echo 2 bytes */
#define CMD_STATUS_SHORT  1  /* query device for status */
#define CMD_RESCAN        2  /* tell the device to rescan the probe bus */
#define CMD_STATUS_LONG   3  /* query device for list of probes and their status */
#define CMD_HARDRESET     4  /* reset device and probe bus */

/* We have to roll our own strerror function here, because the libusb idiots
 * forgot it when they moved to 1.0. 0.X had it, 1.0 got it later after
 * people complained, but it's not in ALL 1.0 versions.
 * The name we use is the 0.X name, so we don't collide with the 1.0
 * name of libusb_strerror in those versions that have it. */
static char * usb_strerror(int e)
{
  static char unknerr[256];

  switch (e) {
    case LIBUSB_SUCCESS:
      return "Success";
    case LIBUSB_ERROR_IO:
      return "Input/output error";
    case LIBUSB_ERROR_INVALID_PARAM:
      return "Invalid parameter";
    case LIBUSB_ERROR_ACCESS:
      return "Access denied (insufficient permissions)";
    case LIBUSB_ERROR_NO_DEVICE:
      return "No such device (it may have been disconnected)";
    case LIBUSB_ERROR_NOT_FOUND:
      return "Entity not found";
    case LIBUSB_ERROR_BUSY:
      return "Resource busy";
    case LIBUSB_ERROR_TIMEOUT:
      return "Operation timed out";
    case LIBUSB_ERROR_OVERFLOW:
      return "Overflow";
    case LIBUSB_ERROR_PIPE:
      return "Pipe error";
    case LIBUSB_ERROR_INTERRUPTED:
      return "System call interrupted (perhaps due to signal)";
    case LIBUSB_ERROR_NO_MEM:
      return "Insufficient memory";
    case LIBUSB_ERROR_NOT_SUPPORTED:
      return "Operation not supported or unimplemented on this platform";
    case LIBUSB_ERROR_OTHER:
      return "Other error";
  };
  snprintf(unknerr, sizeof(unknerr), "Unknown error code %d / 0x%02x", e, e);
  return unknerr;
}

struct daemondata {
  unsigned char serial[6];
  unsigned int port;
  int fd;
  time_t lastseen;
  double lasttemp;
  unsigned char outputformat[1000];
  struct daemondata * next;
};

static void usage(char *name)
{
  printf("usage: %s [-v] [-q] [-d n] [-h] command <parameters>\n", name);
  printf(" -v     more verbose output. can be repeated numerous times.\n");
  printf(" -q     less verbose output. using this more than once will have no effect.\n");
  printf(" -d n   Use the n-th device found (default: 1)\n");
  printf(" -f     relevant for daemon mode only: run in foreground.\n");
  printf(" -h     show this help\n");
  printf("Valid commands are:\n");
  printf(" test     do an echo test with the device\n");
  printf(" status   query and show device status\n");
  printf(" rescan   tell the device to rescan its probe bus\n");
  printf(" reset    tell the device to reset the probe bus and itself\n");
  printf(" showint  shows 'interrupts' received from the device. These are\n");
  printf("          not real interrupts, they are USB 'interrupts', and actually\n");
  printf("          work by polling in regular intervals...\n");
  printf(" daemon   Daemonize and answer queries. This requires one or more\n");
  printf("          parameters in the format 'serial:port', where serial is the serial\n");
  printf("          number of a probe, and port is a TCP port where the data from this\n");
  printf("          probe is to be served, e.g.: affeaffeaf:31337\n");
  printf("          optionally, you can give a third parameter, that specifies how the\n");
  printf("          output to the network should look like, e.g.: affeaffeaf:31337:%%T\n");
  printf("          Available are: %%S = serial of probe, %%T = temperature,\n");
  printf("          %%L = last seen timestamp. The default is '%%S %%T'.\n");
}


void sigpipehandler(int bla) { /* Dummyhandler for catching the event */
  return;
}


static int  usbGetStringAscii(libusb_device_handle *dev, int index, int langid, char *buf, int buflen)
{
  unsigned char buffer[256];
  int           rval, i;

  if ((rval = libusb_control_transfer(dev, LIBUSB_ENDPOINT_IN, LIBUSB_REQUEST_GET_DESCRIPTOR, (LIBUSB_DT_STRING << 8) + index, langid, buffer, sizeof(buffer), 1000)) < 0) {
    return rval;
  }
  if (buffer[1] != LIBUSB_DT_STRING) {
    return 0;
  }
  if ((unsigned char)buffer[0] < rval) {
    rval = (unsigned char)buffer[0];
  }
  /* lossy conversion to ISO Latin1 */
  rval /= 2;
  for (i = 1; i < rval; i++) {
    if (i > buflen) { /* destination buffer overflow */
      break;
    }
    buf[i-1] = buffer[2 * i];
    if (buffer[2 * i + 1] != 0) { /* outside of ISO Latin1 range */
      buf[i-1] = '?';
    }
  }
  buf[i-1] = 0;
  return i-1;
}


static int usbOpenDevice(libusb_device_handle **device, int vendor, char *vendorName, int product, char *productName, int devicerequested)
{
  libusb_device_handle     *handle = NULL;
  libusb_device            **devices;
  struct libusb_device_descriptor desc;
  static int               didUsbInit = 0;
  int                      usbDevices;
  int                      devicesfound = 0;
  int                      i;
  int                      ret;

  if (!didUsbInit) {
    didUsbInit = 1;
    libusb_init(NULL);
  }

  usbDevices = libusb_get_device_list(NULL, &devices);
  if (usbDevices < 0) {
          VERBPRINT(1, "Warning: cannot get USB device list: %s\n", usb_strerror(usbDevices));
  }

  for (i = 0; i < usbDevices; i++) {
    ret = libusb_get_device_descriptor(devices[i], &desc);
    if (ret != 0)
      continue;

    if ((desc.idVendor  == vendor) && (desc.idProduct == product)) {
      char    string[256];
      int     len;
      int     ret;

      ret = libusb_open(devices[i], &handle); /* we need to open the device in order to query strings */
      if (ret != 0) {
        VERBPRINT(1, "Warning: cannot open USB device: %s\n", usb_strerror(ret));
        continue;
      }
      if ((vendorName == NULL) && (productName == NULL)) {  /* name does not matter */
        /* we found it! */
        devicesfound++;
        if (devicesfound == devicerequested) {
          break;
        } else {
          libusb_close(handle);
          handle = NULL;
          continue;
        }
      }
      /* now check whether the names match: */
      len = usbGetStringAscii(handle, desc.iManufacturer, 0x0409, string, sizeof(string));
      if (len < 0) {
        VERBPRINT(1, "Warning: cannot query manufacturer for device: %s\n", usb_strerror(len));
      } else {
        VERBPRINT(3, "seen device from vendor '%s'\n", string);
        if (strcmp(string, vendorName) == 0) {
          len = usbGetStringAscii(handle, desc.iProduct, 0x0409, string, sizeof(string));
          if (len < 0) {
            VERBPRINT(1, "Warning: cannot query product for device: %s\n", usb_strerror(len));
          } else {
            VERBPRINT(3, "seen product '%s'\n", string);
            if (strcmp(string, productName) == 0) {
              devicesfound++;
              if (devicesfound == devicerequested) {
                break;
              }
            }
          }
        }
        libusb_close(handle);
        handle = NULL;
      }
    }
    if (handle) {
      break; /* Stop searching */
    }
  }

  libusb_free_device_list(devices, 1);

  if (handle != NULL) {
    *device = handle;
    return 0;
  }
  return 1;
}

void dodevicetest(libusb_device_handle * handle)
{
  unsigned char	buffer[8];
  unsigned short int i, v, r;
  int nBytes;
  
  VERBPRINT(0, "Doing device test, this may take some time...\n");
  /* The test consists of writing 1000 random numbers to the device and checking
   * the echo. This should discover systematic bit errors (e.g. in bit stuffing).
   */
  for (i = 0; i < 1000; i++){
    VERBPRINT(2, ".");
    v = rand() & 0xffff;
    nBytes = libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, CMD_ECHO, v, 0, buffer, sizeof(buffer), 5000);
    if (nBytes != 2) {
      if (nBytes < 0) {
        fprintf(stderr, "ERROR: USB error: %s\n", usb_strerror(nBytes));
      }
      fprintf(stderr, "ERROR: wrong number of bytes (%d instead of 2) received in iteration %d\n", nBytes, i);
      fprintf(stderr, "value sent = 0x%x\n", v);
      exit(1);
    }
    r = buffer[0] | (buffer[1] << 8);
    if (r != v) {
      fprintf(stderr, "ERROR: data error: received 0x%x instead of 0x%x in iteration %d\n", r, v, i);
      exit(1);
    }
  }
  VERBPRINT(0, "Test succeeded.\n");
  exit(0);
}

static uint32_t make32bit(unsigned char * d) {
  return (d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24));
}

static double maketemp(unsigned char * d) {
  /* old sensor (pre 2014) */  
  int after_point = 0;
  unsigned int temperature_degrees = (unsigned int)d[0]>>1;
		if(d[1]==0x1){
			temperature_degrees *= (-1);
		}
		if(1==(d[0]&0x1)){
			after_point = 5; 
		}else{
			after_point = 0;
		}
  /* Below code for the newer sensor */
  // double res; int32_t t2;
  // t2 = ((d[1] & 0x0F) << 8) + d[0];
  // if (t2 > 0x07FF) {
  //  t2 -= 0x1000;
  // }
  // res = (double)t2 * 0.0625L;
  // return (double) temperature_degrees + ((double) after_point/10.0);
}

static void dodevicestatus(libusb_device_handle * handle) {
  int nBytes; int p; int i;
  unsigned char buffer[1024]; /* That would be enough for 64 probes... */
  uint32_t devicetime;
  unsigned int nprobes;
  
  nBytes = libusb_control_transfer(handle,
                           LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, CMD_STATUS_SHORT,
                           0, 0,
                           buffer, 8,
                           5000);
  if (nBytes < 0) {
    fprintf(stderr, "ERROR: USB error: %s\n", usb_strerror(nBytes));
    exit(1);
  }
  if (nBytes != 8) {
    fprintf(stderr, "ERROR: invalid status received from device (%d bytes)\n", nBytes);
    exit(1);
  }
  printf("Device Version is %02x.%02x\n", buffer[0], buffer[1]);
  devicetime = make32bit(&buffer[2]);
  printf("Device time is currently timestamp %u\n", devicetime);
  nprobes = buffer[6];
  printf("%u probes supported by device\n", nprobes);
  if (nprobes >= ((sizeof(buffer)) / 16)) {
    nprobes = ((sizeof(buffer)) / 16) - 1;
    printf("Warning: Cannot handle this many probes myself - will only show first %u.\n", buffer[6]);
  }
  nBytes = libusb_control_transfer(handle,
                           LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, CMD_STATUS_LONG,
                           0, 0,
                           buffer, (nprobes * 16),
                           5000);
  if ((nBytes % 16) != 0) {
    fprintf(stderr, "ERROR: invalid status received from device (%d bytes)\n", nBytes);
    exit(1);
  }
  if (nBytes != (nprobes * 16)) {
    fprintf(stderr, "WARNING: expected %u probes, got %u.\n", nprobes, (nBytes / 16));
  }
  printf("%2s %-12s %2s %10s %10s %-6s\n", "fa", "serial", "fl", "lastseen", "ticksago", "lastvalue");
  for (p = 0; p < nBytes; p += 16) {
    printf("%02x ", buffer[p+6]);
    for (i = 5; i >= 0; i--) {
      printf("%02x", buffer[p+i]);
    }
    if ((buffer[p+7] & 0x01) == 0x01) { /* in USE */
      printf(" U");
    } else {
      printf("  ");
    }
    if ((buffer[p+7] & 0x02) == 0x02) { /* Parasite powered */
      printf("P");
    } else {
      printf(" ");
    }
    printf(" %10u %10u ", make32bit(&buffer[p+10]),
                          devicetime - make32bit(&buffer[p+10]));
    printf("%6.2lf\n", maketemp(&buffer[p+8]));
  }
}

static void doshowint(libusb_device_handle * handle) {
  int nBytes; int i;
  unsigned char buffer[8];
  
  printf("Waiting for 'interrupts' from the device - abort with ctrl+c\n");
  while (1) {
    i = libusb_interrupt_transfer(handle,
                                LIBUSB_ENDPOINT_IN | 0x01,
                                buffer, sizeof(buffer), &nBytes,
                                0 /* no timeout */ );
    if (i != 0) {
      fprintf(stderr, "ERROR: USB error: %s\n", usb_strerror(i));
      exit(1);
    }
    if (nBytes != 8) {
      fprintf(stderr, "ERROR: invalid interrupt message received from device (%d bytes instead of 8) - continuing\n", nBytes);
      continue;
    }
    {
      char tbuf[100];
      time_t tt;
      tt = time(NULL);
      strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", localtime(&tt));
      printf("[%s] ", tbuf);
    }
    printf("Data: probeserial='");
    for (i = 5; i >= 0; i--) {
      printf("%02x", buffer[i]);
    }
    printf("' temp='%6.2lf'\n", maketemp(&buffer[6]));
  }
  /* never reached */
}

void logaccess(struct sockaddr * soa, int soalen, char * txt) {
  struct sockaddr_in * sav4;
  struct sockaddr_in6 * sav6;

  if (soalen == sizeof(struct sockaddr_in6)) {
    sav6 = (struct sockaddr_in6 *)soa;
    if ((sav6->sin6_addr.s6_addr[ 0] == 0)
     && (sav6->sin6_addr.s6_addr[ 1] == 0)
     && (sav6->sin6_addr.s6_addr[ 2] == 0)
     && (sav6->sin6_addr.s6_addr[ 3] == 0)
     && (sav6->sin6_addr.s6_addr[ 4] == 0)
     && (sav6->sin6_addr.s6_addr[ 5] == 0)
     && (sav6->sin6_addr.s6_addr[ 6] == 0)
     && (sav6->sin6_addr.s6_addr[ 7] == 0)
     && (sav6->sin6_addr.s6_addr[ 8] == 0)
     && (sav6->sin6_addr.s6_addr[ 9] == 0)
     && (sav6->sin6_addr.s6_addr[10] == 0xFF)
     && (sav6->sin6_addr.s6_addr[11] == 0xFF)) {
      /* This is really a IPv4 not a V6 access, so log it as
       * a such. */
      VERBPRINT(2, "%d.%d.%d.%d\t%s\n", sav6->sin6_addr.s6_addr[12],
              sav6->sin6_addr.s6_addr[13],
              sav6->sin6_addr.s6_addr[14],
              sav6->sin6_addr.s6_addr[15], txt);
    } else {
      /* True IPv6 access */
      VERBPRINT(2, "%x:%x:%x:%x:%x:%x:%x:%x\t%s\n",
              (sav6->sin6_addr.s6_addr[ 0] << 8) | sav6->sin6_addr.s6_addr[ 1],
              (sav6->sin6_addr.s6_addr[ 2] << 8) | sav6->sin6_addr.s6_addr[ 3],
              (sav6->sin6_addr.s6_addr[ 4] << 8) | sav6->sin6_addr.s6_addr[ 5],
              (sav6->sin6_addr.s6_addr[ 6] << 8) | sav6->sin6_addr.s6_addr[ 7],
              (sav6->sin6_addr.s6_addr[ 8] << 8) | sav6->sin6_addr.s6_addr[ 9],
              (sav6->sin6_addr.s6_addr[10] << 8) | sav6->sin6_addr.s6_addr[11],
              (sav6->sin6_addr.s6_addr[12] << 8) | sav6->sin6_addr.s6_addr[13],
              (sav6->sin6_addr.s6_addr[14] << 8) | sav6->sin6_addr.s6_addr[15],
              txt);
    }
  } else if (soalen == sizeof(struct sockaddr_in)) {
    unsigned char brokeni32[4];

    sav4 = (struct sockaddr_in *)soa;
    brokeni32[0] = (sav4->sin_addr.s_addr & 0xFF000000UL) >> 24;
    brokeni32[1] = (sav4->sin_addr.s_addr & 0x00FF0000UL) >> 16;
    brokeni32[2] = (sav4->sin_addr.s_addr & 0x0000FF00UL) >>  8;
    brokeni32[3] = (sav4->sin_addr.s_addr & 0x000000FFUL) >>  0;
    VERBPRINT(2, "%d.%d.%d.%d\t%s\n", brokeni32[0], brokeni32[1],
            brokeni32[2], brokeni32[3], txt);
  } else {
    VERBPRINT(2, "!UNKNOWN_ADDRESS_TYPE!\t%s\n", txt);
  }
}


static void printtooutbuf(char * outbuf, int oblen, struct daemondata * dd) {
  unsigned char * pos = &dd->outputformat[0];
  while (*pos != 0) {
    if (*pos == '%') {
      pos++;
      if        (*pos == 'S') { /* Serial */
        int i;
        for (i = 5; i >= 0; i--) {
          outbuf += sprintf(outbuf, "%02x", dd->serial[i]);
        }
      } else if ((*pos == 'T') || (*pos == 't')) { /* Temperature */
        if ((dd->lastseen + 60) < time(NULL)) { /* Stale data / no data yet */
          outbuf += sprintf(outbuf, "%s", "N/A");
          //outbuf += sprintf(outbuf, "%.2lf", dd->lasttemp);
        } else {
          if (*pos == 'T') { /* fixed width */
            outbuf += sprintf(outbuf, "%6.2lf", dd->lasttemp);
          } else { /* variable width. */
            outbuf += sprintf(outbuf, "%.2lf", dd->lasttemp);
          }
        }
      } else if (*pos == 'r') { /* carriage return */
        *outbuf = '\r';
        outbuf++;
      } else if (*pos == 'n') { /* linefeed / Newline */
        *outbuf = '\n';
        outbuf++;
      } else if (*pos == 'L') { /* Last seen */
        outbuf += sprintf(outbuf, "%u", (unsigned int)dd->lastseen);
      } else if (*pos == 0) {
        *outbuf = 0;
        return;
      }
      pos++;
    } else {
      *outbuf = *pos;
      outbuf++;
      pos++;
    }
  }
  *outbuf = 0;
}

static void dotryrestart(struct daemondata * dd, char ** argv) {
  struct daemondata * curdd = dd;
  
  if (!restartonerror) { return; }
  /* close all open sockets */
  while (curdd != NULL) {
    close(curdd->fd);
    curdd = curdd->next;
  }
  fprintf(stderr, "Will try to restart in %d second(s)...\n", restartonerror);
  sleep(restartonerror);
  execv(argv[0], argv);
}

static void dodaemon(libusb_device_handle * handle, struct daemondata * dd, char ** argv) {
  fd_set mylsocks;
  struct daemondata * curdd;
  struct timeval to;
  int nBytes;
  int ret;
  unsigned char buffer[8];
  int maxfd;
  int readysocks;
  
  while (1) {
    do {
      curdd = dd; /* Start from beginning */
      maxfd = 0;
      FD_ZERO(&mylsocks);
      while (curdd != NULL) {
        FD_SET(curdd->fd, &mylsocks);
        if (curdd->fd > maxfd) { maxfd = curdd->fd; }
        curdd = curdd->next;
      }
      to.tv_sec = 0; to.tv_usec = 1;
      if ((readysocks = select((maxfd + 1), &mylsocks, NULL, NULL, &to)) < 0) { /* Error?! */
        if (errno != EINTR) {
          perror("ERROR: error on select()");
          dotryrestart(dd, argv);
          exit(1);
        }
      } else {
        curdd = dd;
        while (curdd != NULL) {
          if (FD_ISSET(curdd->fd, &mylsocks)) {
            int tmpfd;
            struct sockaddr_in6 srcad;
            socklen_t adrlen = sizeof(srcad);
            tmpfd = accept(curdd->fd, (struct sockaddr *)&srcad, &adrlen);
            if (tmpfd < 0) {
              perror("WARNING: Failed to accept() connection");
            } else {
              char outbuf[250];
              printtooutbuf(outbuf, strlen(outbuf), curdd);
              logaccess((struct sockaddr *)&srcad, adrlen, outbuf);
              write(tmpfd, outbuf, strlen(outbuf));
              close(tmpfd);
            }
          }
          curdd = curdd->next;
        }
      }
    } while (readysocks > 0);
    /* Now check if there is anything available on USB */
    ret = libusb_interrupt_transfer(handle,
                                LIBUSB_ENDPOINT_IN | 0x01,
                                buffer, sizeof(buffer), &nBytes,
                                100 /* short timeout - but can't be shorter, else no data is seen? strange. */ );
    if (ret != 0) { /* error reported by libusb */
      /* As nice as libusb is, the documentation is just HORROR. And so is
       * this interface: This is the best way I have found to check if the
       * error is actually a timeout. I would expect something like an
       * usb_errno variable one can poll, but no... */
      if (ret != LIBUSB_ERROR_TIMEOUT) {
        fprintf(stderr, "ERROR: USB error: %s\n", usb_strerror(ret));
        dotryrestart(dd, argv);
        exit(1);
      }
    } else if (nBytes != 8) { /* No error but wrong number of bytes */
      fprintf(stderr, "WARNING: invalid interrupt message received from device (%d bytes instead of 8) - continuing\n", nBytes);
    } else {
      VERBPRINT(3, "interrupt data received from device %02x%02x%02x%02x%02x%02x ",
                   buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);
      curdd = dd;
      while (curdd != NULL) {
        if (memcmp(&curdd->serial[0], &buffer[0], 6) == 0) {
          /* This belongs here - update! */
          curdd->lasttemp = maketemp(&buffer[6]);
          VERBPRINT(3, "temp = %.2f\n", curdd->lasttemp);
          curdd->lastseen = time(NULL);
        }
        curdd = curdd->next;
      }
    }
    if (restartonerror) { /* We're supposed to restart on errors */
      /* so check if we haven't had updates from a probe for a long time */
      curdd = dd;
      while (curdd != NULL) {
        if (curdd->lastseen != 0) {
          if ((time(NULL) - curdd->lastseen) > 300) {
            /* No update for 300 seconds - that's bad. */
            /* Send reset command, then try a restart. */
            nBytes = libusb_control_transfer(handle,
                                     LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN,
                                     CMD_HARDRESET, 0, 0,
                                     buffer, sizeof(buffer), 5000);
            dotryrestart(dd, argv);
          }
        }
        curdd = curdd->next;
      }
    }
  }
  /* never reached */
}

int main(int argc, char ** argv)
{
  libusb_device_handle * handle = NULL;
  unsigned char buffer[8];
  int nBytes;
  int curarg;
  int ret;

  for (curarg = 1; curarg < argc; curarg++) {
    if        (strcmp(argv[curarg], "-v") == 0) {
      verblev++;
    } else if (strcmp(argv[curarg], "-q") == 0) {
      verblev--;
    } else if (strcmp(argv[curarg], "-f") == 0) {
      runinforeground = 1;
    } else if (strcmp(argv[curarg], "-h") == 0) {
      usage(argv[0]); exit(0);
    } else if (strcmp(argv[curarg], "--help") == 0) {
      usage(argv[0]); exit(0);
    } else if (strcmp(argv[curarg], "--restartonerror") == 0) {
      restartonerror += 5;
    } else if (strcmp(argv[curarg], "-d") == 0) {
      curarg++;
      if (curarg >= argc) {
        fprintf(stderr, "ERROR: -d requires a numeric parameter >= 1!\n");
        usage(argv[0]); exit(1);
      }
      devicenr = strtoul(argv[curarg], NULL, 10);
      if (devicenr <= 0) {
        fprintf(stderr, "ERROR: -d requires a numeric parameter >= 1!\n");
        usage(argv[0]); exit(1);
      }
    } else {
      /* Unknown - must be the command. */
      break;
    }
  }
  if (curarg == argc) {
    fprintf(stderr, "ERROR: No command given!\n");
    usage(argv[0]);
    exit(1);
  }
  libusb_init(NULL);
  if (usbOpenDevice(&handle, USBDEV_SHARED_VENDOR, "www.poempelfox.de", USBDEV_SHARED_PRODUCT, "ds1820tousb", devicenr) != 0) {
    fprintf(stderr, "ERROR: Could not find the ds1820tousb device nr. %d on the USB.\n", devicenr);
    exit(1);
  }
  ret = libusb_claim_interface(handle, 0);
  if (ret < 0) {
    fprintf(stderr, "ERROR: Failed to claim ds1820tousb device nr. %d on the USB: %s\n", devicenr, usb_strerror(ret));
    exit(1);
  }
  if        (strcmp(argv[curarg], "test") == 0) {
    dodevicetest(handle);
  } else if (strcmp(argv[curarg], "status") == 0) {
    dodevicestatus(handle);
  } else if (strcmp(argv[curarg], "rescan") == 0) {
    nBytes = libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, CMD_RESCAN, 0, 0, buffer, sizeof(buffer), 5000);
    if (nBytes < 0) {
      fprintf(stderr, "ERROR: USB error: %s\n", usb_strerror(nBytes));
    } else {
      if (nBytes == 1) {
        switch(buffer[0]) {
        case 23:
          VERBPRINT(0, "Device responded: OK, scheduling rescan\n");
          break;
        case 42:
          VERBPRINT(0, "Device responded: rescan already scheduled or in progress\n");
          break;
        default:
          fprintf(stderr, "ERROR: Invalid reply to rescan command from device (unknown status %02x)\n", buffer[0]);
          break;
        };
      } else {
        fprintf(stderr, "ERROR: Invalid reply to rescan command from device (%d bytes instead of 1)\n", nBytes);
      }
    }
  } else if (strcmp(argv[curarg], "reset") == 0) {
    nBytes = libusb_control_transfer(handle, LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN, CMD_HARDRESET, 0, 0, buffer, sizeof(buffer), 5000);
    VERBPRINT(1, "%d bytes received as reply.\n", nBytes);
    VERBPRINT(0, "%s\n", "Reset command sent. If this worked, you should see an USB disconnect and reconnect now\n");
  } else if (strcmp(argv[curarg], "showint") == 0) { /* More of a debug mode: Show interrupt data */
    doshowint(handle);
  } else if (strcmp(argv[curarg], "daemon") == 0) { /* Daemon mode */
    struct daemondata * mydaemondata = NULL;
    curarg++;
    do {
      int l; int optval;
      struct daemondata * newdd;
      struct sockaddr_in6 soa;
      
      if (curarg >= argc) continue;
      newdd = calloc(sizeof(struct daemondata), 1);
      newdd->next = mydaemondata;
      mydaemondata = newdd;
      l = sscanf(argv[curarg], "%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx:%u:%999[^\n]",
                 &mydaemondata->serial[5], &mydaemondata->serial[4], &mydaemondata->serial[3], &mydaemondata->serial[2],
                 &mydaemondata->serial[1], &mydaemondata->serial[0], &mydaemondata->port, &mydaemondata->outputformat[0]);
      if (l < 7) {
        fprintf(stderr, "ERROR: failed to parse daemon command parameter '%s'\n", argv[curarg]);
        exit(1);
      }
      if (l == 7) {
        strcpy((char *)&mydaemondata->outputformat[0], "%S %T");
      }
      /* Open the port */
      mydaemondata->fd = socket(PF_INET6, SOCK_STREAM, 0);
      soa.sin6_family = AF_INET6;
      soa.sin6_addr = in6addr_any;
      soa.sin6_port = htons(mydaemondata->port);
      optval = 1;
      if (setsockopt(mydaemondata->fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval))) {
        VERBPRINT(0, "WARNING: failed to setsockopt REUSEADDR: %s", strerror(errno));
      }
#ifdef BRAINDEADOS
      /* For braindead operating systems in default config (BSD, Windows,
       * newer Debian), we need to tell the OS that we're actually fine with
       * accepting V4 mapped addresses as well. Because apparently for
       * braindead idiots accepting only selected addresses is a more default
       * case than accepting everything. */
      optval = 0;
      if (setsockopt(mydaemondata->fd, IPPROTO_IPV6, IPV6_V6ONLY, &optval, sizeof(optval))) {
        VERBPRINT(0, "WARNING: failed to setsockopt IPV6_V6ONLY: %s", strerror(errno));
      }
#endif
      if (bind(mydaemondata->fd, (struct sockaddr *)&soa, sizeof(soa)) < 0) {
        perror("Bind failed");
        printf("Could not bind to port %u\n", mydaemondata->port);
        exit(1);
      }
      if (listen(mydaemondata->fd, 20) < 0) { /* Large Queue as we block for some time while reading USB! */
        perror("Listen failed");
        exit(1);
      }
      curarg++;
    } while (curarg < argc);
    if (mydaemondata == NULL) {
      fprintf(stderr, "ERROR: the daemon command requires parameters.\n");
      exit(1);
    }
    /* the good old doublefork trick from 'systemprogrammierung 1' */
    if (runinforeground != 1) {
      int ourpid;
      VERBPRINT(2, "launching into the background...\n");
      ourpid = fork();
      if (ourpid < 0) {
        perror("Ooops, fork() #1 failed");
        exit(1);
      }
      if (ourpid == 0) { /* We're the child */
        ourpid = fork(); /* fork again */
        if (ourpid < 0) {
          perror("Ooooups. fork() #2 failed");
          exit(1);
        }
        if (ourpid == 0) { /* Child again */
          /* Just don't exit, we'll continue below. */
        } else { /* Parent */
          exit(0); /* Just exit */
        }
      } else { /* Parent */
        exit(0); /* Just exit */
      }
    }
    {
      struct sigaction sia;
      sia.sa_handler = sigpipehandler;
      sigemptyset(&sia.sa_mask); /* If we don't do this, we're likely */
      sia.sa_flags = 0;          /* to die from 'broken pipe'! */
      sigaction(SIGPIPE, &sia, NULL);
    }
    dodaemon(handle, mydaemondata, argv);
  } else {
    fprintf(stderr, "ERROR: Command '%s' is unknown.\n", argv[curarg]);
    usage(argv[0]);
    exit(1);
  }
  libusb_close(handle);
  return 0;
}
