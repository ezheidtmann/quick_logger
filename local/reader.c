#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>   /* UNIX standard function definitions */
#include <termios.h>  /* POSIX terminal control definitions */
#include <sys/ioctl.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>

#include "../record.h"

#define HANDSHAKE_MAX_ATTEMPTS  15 // # of handshakes to attempt
#define HANDSHAKE_WAIT_MSECS   100 // milliseconds to wait for response

int main (int argc, char** argv) {
  struct record r;
  int port;

  /*printf("long: %d, int: %d, short: %d, char: %d\n",
  sizeof(long), sizeof(int), sizeof(short), sizeof(char));
  printf("record: %d\n", sizeof(struct record));*/

  port = serialport_init(argv[1], EZH_BAUD);

  if (port < 0) {
    printf("Failed to open file!\n");
    return;
  }

  if (!handshake(port)) {
    printf("Failed to handshake with Arduino.\n");
    return;
  }

  int n;
  while (1) {
    if (readlen(port, &r, sizeof(struct record), 1000)) {
      printf("%lu, %u\n", r.ms, r.val);
    }
  }
}

/**
 Perform a handshake with the Arduino

 We send SYN packets with a random number and wait for ACK packets
 containing that number.
 */
int handshake(int fd) {
  int n;
  // syn packet: 7 bytes of text, 1 random byte
  char syn[8] = "EZH SYN";
  syn[7] = (char) random();

  // ack packet: 7 bytes of text, same random byte
  char ack[8] = "EZH ACK";
  ack[7] = syn[7];

  char ready[9] = "EZHREADY";

  int num_sent;
  for (num_sent = 0; num_sent < HANDSHAKE_MAX_ATTEMPTS; num_sent++) {
    // send SYN
    n = write(fd, syn, 8);
    if (n != 8) {
      perror("handshake: failed to write SYN ");
    }
    // receive ACK
    n = waitFor(fd, ack, 8, HANDSHAKE_WAIT_MSECS);
    if (n > 0) { // got it!
      write(fd, ready, 8);
      return 1;
    }
  }

  return 0;
}

int readlen(int fd, char* buf, int len, unsigned long timeout_msecs) {
  return _wait(fd, buf, len, timeout_msecs, 0);
}

int waitFor(int fd, char* key, int key_len, unsigned long timeout_msecs) {
  return _wait(fd, key, key_len, timeout_msecs, 1);
}

/**
 Wait for input, either matching a key or filling a buffer

 cmp: If 1, search for key in input stream. If 0, fill buffer at key.
 fd: a valid file descriptor representing the input stream
 key: If cmp == 1, search for these bytes in the input stream
 key_len: Number of bytes in key to search for, or to fill
 timeout_msecs: Number of milliseconds to wait, at maximum. 
   Execution of this funtion will take no more than about this long.
 */
int _wait(int fd, char* key, int key_len, unsigned long timeout_msecs, int cmp) {
  struct timeval t_end, t_now, t_timeout;

  // convert msecs to timeval
  t_timeout.tv_sec = timeout_msecs / 1000;
  t_timeout.tv_usec = (timeout_msecs % 1000) * 1000;

  // set t_end = t_now + t_timeout
  gettimeofday(&t_now, NULL);
  timeradd(&t_now, &t_timeout, &t_end);

  // for select: watch just the one fd
  int nfds = fd + 1;
  fd_set reading;
  FD_SET(fd, &reading);

  // initialize buffer. if we're comparing, we need a new one.
  // otherwise, we need to use the one supplied.
  char* buf;
  if (cmp) {
    buf = (char*) malloc(key_len);
  }
  else {
    buf = key;
  }

  int bytes_read;
  int i = 0;
  int fds_available;
  do {
    // wait for data on fd, ensuring we do not exceed total timeout
    timersub(&t_end, &t_now, &t_timeout);
    fds_available = select(nfds, &reading, NULL, NULL, &t_timeout);

    if (!fds_available) { // we timed out
      if (cmp) free(buf);
      return 0;
    }

    // read from the fd, hopefully filling the buffer
    // we have to read 1 byte at a time because we don't want to miss
    // the start of this string.
    bytes_read = read(fd, buf + i, 1);

    #ifdef DEBUG
    if (bytes_read) {
      printf("read: buf[%d] = %c [%d]\n", i, buf[i], buf[i]);
    }
    #endif

    // update writing index 
    i += bytes_read;

    // if buf != key, reset indices
    if (cmp && strncmp(buf, key, i) != 0) {
      i = 0;
    }

    // yay! we matched.
    if (i == key_len)  {
      if (cmp) free(buf);
      return 1;
    }

    gettimeofday(&t_now, NULL);
  } while (timercmp(&t_end, &t_now, >));

  // we've read data, but it didn't match.
  if (cmp) free(buf);
  return 0;
}

// serialport_init(): from arduino-serial.c by Tod E. Kurt, tod@todbot.com
// takes the string name of the serial port (e.g. "/dev/tty.usbserial","COM1")
// and a baud rate (bps) and connects to that port at that speed and 8N1.
// opens the port in fully raw mode so you can send binary data.
// returns valid fd, or -1 on error
int serialport_init(const char* serialport, int baud)
{
  struct termios toptions;
  int fd;

  //fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
  //        serialport,baud);

  fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1)  {
    perror("init_serialport: Unable to open ");
    return -1;
  }

  if (tcgetattr(fd, &toptions) < 0) {
    perror("init_serialport: Couldn't get term attributes");
    return -1;
  }

  speed_t brate = baud; // let you override switch below if needed
  switch (baud) {
    case 4800:   brate=B4800;   break;
    case 9600:   brate=B9600;   break;
    case 19200:  brate=B19200;  break;
    case 38400:  brate=B38400;  break;
    case 57600:  brate=B57600;  break;
    case 115200: brate=B115200; break;
  }
  cfsetispeed(&toptions, brate);
  cfsetospeed(&toptions, brate);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // no flow control
  toptions.c_cflag &= ~CRTSCTS;

  toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  toptions.c_oflag &= ~OPOST; // make raw

  // see: http://unixwiz.net/techtips/termios-vmin-vtime.html
  toptions.c_cc[VMIN]  = 0;
  toptions.c_cc[VTIME] = 20;

  if (tcsetattr(fd, TCSANOW, &toptions) < 0) {
    perror("init_serialport: Couldn't set term attributes");
    return -1;
  }

  return fd;
}
