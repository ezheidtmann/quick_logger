#include "record.h"

#define BUFFER_SIZE 250

int quit = 0;

void setup() {
  Serial.begin(EZH_BAUD);

  buf_init(BUFFER_SIZE);
  pinMode(13, OUTPUT);

  if (!handshake(Serial)) {
    quit = 1;
  }  
}

long last_sync = 0;

void loop() {
  struct record r;

  if (quit) {
    digitalWrite(13, HIGH);
    delay(100);
    digitalWrite(13, LOW);
    delay(100);
    return;
  }

  // take a unit of data!
  r.val = analogSum(A0, 7);
  r.ms = micros(); // read after analogSum for good reading

  // if the buffer is full, write it.
  if (!buf_add(&r)) {
    // signal that we're writing
    digitalWrite(13, HIGH);
    buf_write(Serial);
    buf_add(&r); // don't forget to add that data point!
    digitalWrite(13, LOW);
  }
}

/**
 * Repeatedly read an analog pin and return the sum of all reads
 * 
 * Divide the result by num to find the average
 */
int analogSum(int pin, int num) {
  int i, sum;
  sum = 0;
  for (i = 0; i < num; ++i) {
    sum += analogRead(A0);
  }
  return sum;
}

/**
 * Perform the Arduino side of the handshake.
 * 
 * Send a help message, then wait for "EZH SYN" followed by a
 * byte. Send "EZH ACK" followed by that same byte. Wait for
 * "EZHREADY" and then return.
 */
int handshake(HardwareSerial out) {
  int n;
  char ack[8] = "EZH ACK";

  while (1) {
    // Send help message for ASCII readers
    out.println("Welcome to the EZH data logger.");
    out.println("Data is binary; please use a reader.");
    out.println(sizeof(struct record));
#ifdef __i386__
    out.println("I am a computer.");
#endif

    // Wait for SYN, then send ACK
    if (waitFor(out, "EZH SYN", 7, 10000) > 0) {
      while (!out.available()) {
      }
      ack[7] = out.read();
      out.write((byte*) ack, 8);
      while (!waitFor(out, "EZHREADY", 8, 10000)) {
      }
      return 1;
    }
  }
}

int waitFor(HardwareSerial out, char* key, int key_len, int timeout_msecs) {
  int end_msecs = millis() + timeout_msecs;
  char* buf = (char*) malloc(key_len);
  int n, i;

  i = 0;
  while (millis() < end_msecs) {
    n = out.read();
    if (n != -1) {
      buf[i++] = (char) n;
      if (strncmp(buf, key, i) != 0) {
        i = 0;
      }
      if (i == key_len) {
        return 1;
      }
    }
  }
  return 0;
}
