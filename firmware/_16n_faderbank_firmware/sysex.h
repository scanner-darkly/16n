// SYSEX channels for each fader
const int sysex_channels[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};

#if TRANSMIT_SYSEX == ROLAND_MKS50

static uint8_t mks50Groups[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static uint8_t mks50Params[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
static uint8_t mksSysExData[] = { 0xF0, 0x41, 0x36, 0x0, 0x23, 0x20, 0x01, 0x0, 0x0, 0xF7 };

uint8_t* buildSysExMessage(uint8_t channel, uint8_t fader, uint8_t value) {
  mksSysExData[3] = channel - 1;
  mksSysExData[5] = mks50Groups[fader];
  mksSysExData[7] = mks50Params[fader];
  mksSysExData[8] = value;
  return mksSysExData;
}

int getSysExMessageLength(uint8_t channel, uint8_t fader, uint8_t value) {
  return 10;
}

#elif TRANSMIT_SYSEX == SOMETHING_ELSE

// add new implementations here

#else

uint8_t* buildSysExMessage(uint8_t channel, uint8_t fader, uint8_t value) { return 0; }
int getSysExMessageLength(uint8_t channel, uint8_t fader, uint8_t value) { return 0; }

#endif
