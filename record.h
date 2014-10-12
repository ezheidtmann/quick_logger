#define EZH_BAUD 115200

struct record {
  unsigned long ms;
  unsigned short val;
#ifdef __i386__
} __attribute__((packed));
#else
};
#endif
