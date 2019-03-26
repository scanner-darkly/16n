/*
 * 16n Faderbank Firmware
 * (c) 2017,2018 by Brian Crabtree, Sean Hellfritsch, Tom Armitage, and Brendon Cassidy
 * MIT License
 */

/*
 * NOTES:
 * - Hardware MIDI is on pin 1
 * - You **must** also compile this with Tools->USB type set to MIDI or MIDI/Serial (for debugging)
 * - You also should overclock to 120MHz to make it as snappy as possible
 */

/*
 * ALL configuration should take place in config.h.
 * You can disable/enable flags, and configure  MIDI channels in there.
 */

#include "config.h"
#include <i2c_t3.h>
#include <MIDI.h>
#include <ResponsiveAnalogRead.h>
#include <CD74HC4067.h>

#include "TxHelper.h"

const int maxCCCount = 128;

MIDI_CREATE_DEFAULT_INSTANCE();

// loop helpers
int i, temp;

// midi write helpers
int q, shiftyTemp, notShiftyTemp;

// the storage of the values; current is in the main loop; last value is for midi output
int volatile currentValue[channelCount];
int lastMidiValue[channelCount];

int lastCCValue[channelCount][maxCCCount];

#ifdef MASTER

// memory of the last unshifted value
int lastValue[channelCount];

// the i2c message buffer we are sending
uint8_t messageBuffer[6];

// temporary values
uint16_t valueTemp;
uint8_t device = 0;
uint8_t port = 0;
uint8_t jfMode = 0;

#endif

// MIDI notes
int midiPitch[POLYPHONY];
int midiVelocity[POLYPHONY];
int midiState[POLYPHONY];

// the thing that smartly smooths the input
ResponsiveAnalogRead *analog[channelCount];

// mux config
CD74HC4067 mux(8, 7, 6, 5);
#ifdef REV
const int muxMapping[16] = {8, 9, 10, 11, 12, 13, 14, 15, 7, 6, 5, 4, 3, 2, 1, 0};
#else
const int muxMapping[16] = {0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8};
#endif

// MIDI timers
IntervalTimer midiWriteTimer;
IntervalTimer midiReadTimer;
int midiInterval = 1000; // 1ms
bool shouldDoMidiRead = false;
bool shouldDoMidiWrite = false;

// helper values for i2c reading and future expansion
int activeInput = 0;
int activeMode = 0;
int activeChannel = 0;
int activeCC = 0;
int noteChannel = 0;
int note = 0;
int noteVelocity = 0;
int sendNote = 0;

/*
 * The function that sets up the application
 */
void setup()
{

#ifdef DEBUG
  while (!Serial)
    ;
  Serial.print("16n Firmware Debug Mode\n");
#endif

// initialize the TX Helper
#ifdef V125
  TxHelper::UseWire1(true);
#else
  TxHelper::UseWire1(false);
#endif
  TxHelper::SetPorts(16);
  TxHelper::SetModes(4);

  // set read resolution to teensy's 13 usable bits
  analogReadResolution(13);

  // initialize the value storage
  for (i = 0; i < channelCount; i++)
  {
    // analog[i] = new ResponsiveAnalogRead(0, false);

    analog[i] = new ResponsiveAnalogRead(0, true, .0001);
    analog[i]->setAnalogResolution(1 << 13);
    currentValue[i] = 0;
    lastMidiValue[i] = 0;
#ifdef MASTER
    lastValue[i] = 0;
#endif
  }

// i2c using the default I2C pins on a Teensy 3.2
#ifdef MASTER

#ifdef DEBUG
  Serial.println("Enabling i2c in MASTER mode");
#endif

#ifdef V125
  Wire1.begin(I2C_MASTER, I2C_ADDRESS, I2C_PINS_29_30, I2C_PULLUP_EXT, 400000);
#else
  Wire.begin(I2C_MASTER, I2C_ADDRESS, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
#endif

#else
  // non-master mode

#ifdef DEBUG
  Serial.println("Enabling i2c enabled in SLAVE mode");
#endif

#ifdef V125
  Wire1.begin(I2C_SLAVE, I2C_ADDRESS, I2C_PINS_29_30, I2C_PULLUP_EXT, 400000);
  Wire1.onReceive(i2cWrite);
  Wire1.onRequest(i2cReadRequest);
#else
  Wire.begin(I2C_SLAVE, I2C_ADDRESS, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  Wire.onReceive(i2cWrite);
  Wire.onRequest(i2cReadRequest);
#endif

#endif

  // turn on the MIDI party
  MIDI.begin();
  MIDI.setHandleControlChange(midiHandleControlChange);
  MIDI.setHandleNoteOn(midiHandleNoteOn);
  MIDI.setHandleNoteOff(midiHandleNoteOff);
  usbMIDI.setHandleControlChange(midiHandleControlChange);
  usbMIDI.setHandleNoteOn(midiHandleNoteOn);
  usbMIDI.setHandleNoteOff(midiHandleNoteOff);
  midiWriteTimer.begin(writeMidi, midiInterval);
  midiReadTimer.begin(readMidi, midiInterval);

#ifdef LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
#endif

  for (int i = 0; i < POLYPHONY; i++) midiState[i] = 0;
}

/*
 * The main read loop that goes through all of the sliders
 */
void loop()
{
  // read loop using the i counter
  for (i = 0; i < channelCount; i++)
  {
#ifdef V125
    temp = analogRead(ports[i]); // mux goes into A0
#else
    // set mux to appropriate channel
    mux.channel(muxMapping[i]);

    // read the value
    temp = analogRead(0); // mux goes into A0
#endif

    // put the value into the smoother
    analog[i]->update(temp);

    // read from the smoother, constrain (to account for tolerances), and map it
    temp = analog[i]->getValue();

#ifdef FLIP
    temp = MAXFADER - temp;
#endif

    temp = constrain(temp, MINFADER, MAXFADER);

    temp = map(temp, MINFADER, MAXFADER, 0, 16383);

    // map and update the value
    currentValue[i] = temp;
  }

  if (shouldDoMidiRead)
  {
    doMidiRead();
    noInterrupts();
    shouldDoMidiRead = false;
    interrupts();
  }

  if (shouldDoMidiWrite)
  {
    doMidiWrite();
    noInterrupts();
    shouldDoMidiWrite = false;
    interrupts();
  }

#ifdef MASTER
  for (int i = 0; i < POLYPHONY; i++) {
    if (midiState[i] == -1)
    {
#ifdef JFMIDI
      sendJFNote(i, midiPitch[i], midiVelocity[i]);
#endif
#ifdef ER301MIDI
      sendER301Note(i, midiPitch[i], midiVelocity[i]);
#endif
      midiState[i] = midiVelocity[i] ? 1 : 0;
    }
  }
#endif
}

void midiHandleControlChange(byte channel, byte control, byte value)
{
    if (channel >= channelCount || control >= maxCCCount) return;
    lastCCValue[channel][control] = value;
}

void midiHandleNoteOn(uint8_t channel, uint8_t note, uint8_t velocity)
{
#ifdef DEBUG
  Serial.printf("Received MIDI note on channel: %d note: %d vel: %d\n", channel, note, velocity);
#endif
  addNote(channel, note, velocity);
}

void midiHandleNoteOff(uint8_t channel, uint8_t note, uint8_t velocity)
{
#ifdef DEBUG
  Serial.printf("Received MIDI note off channel: %d note: %d vel: %d\n", channel, note, velocity);
#endif
  removeNote(channel, note, velocity);
}

void addNote(uint8_t channel, uint8_t note, uint8_t velocity)
{
  int index = -1;
  int oldest = -1;
  for (int i = 0; i < POLYPHONY; i++) {
    if (midiState[i] == 0)
    {
      midiPitch[i] = note;
      midiVelocity[i] = velocity;
      midiState[i] = -1;
      index = -1;
#ifdef DEBUG
      Serial.printf("Note added to voice  %d\n", i);
#endif
      break;
    }
    if (midiState[i] > oldest)
    {
      index = i;
      oldest = midiState[i];
    }
  }
  if (index != -1)
  {
    for (int i = 0; i < POLYPHONY; i++) {
      if (midiState[i] > 0) midiState[i]++;
    }
    midiPitch[index] = note;
    midiVelocity[index] = velocity;
    midiState[index] = -1;
#ifdef DEBUG
    Serial.printf("Note added to voice  %d\n", index);
#endif
  }
}

void removeNote(uint8_t channel, uint8_t note, uint8_t velocity)
{
  for (int i = 0; i < POLYPHONY; i++) {
    if (midiPitch[i] == note)
    {
      midiVelocity[i] = 0;
      midiState[i] = -1;
      break;
    }
  }
}

/*
 * Tiny function called via interrupt
 * (it's important to catch inbound MIDI messages even if we do nothing with
 * them.)
 */
void readMidi()
{
  shouldDoMidiRead = true;
}

/*
 * Function called when shouldDoMidiRead flag is HIGH
 */

void doMidiRead()
{
  MIDI.read();
  usbMIDI.read();
}

/*
 * Tiny function called via interrupt
 */
void writeMidi()
{
  shouldDoMidiWrite = true;
}

/*
 * The function that writes changes in slider positions out the midi ports
 * Called when shouldDoMidiWrite flag is HIGH
 */
void doMidiWrite()
{
  // write loop using the q counter (
  // (can't use i or temp cuz this might interrupt the reads)
  for (q = 0; q < channelCount; q++)
  {
    notShiftyTemp = currentValue[q];

    // shift for MIDI precision (0-127)
    shiftyTemp = notShiftyTemp >> 7;

    // if there was a change in the midi value
    if (shiftyTemp != lastMidiValue[q])
    {
      // send the message over USB and physical MIDI
      usbMIDI.sendControlChange(usb_ccs[q], shiftyTemp, usb_channels[q]);
      MIDI.sendControlChange(trs_ccs[q], shiftyTemp, trs_channels[q]);

      // store the shifted value for future comparison
      lastMidiValue[q] = shiftyTemp;

#ifdef DEBUG
      Serial.printf("MIDI[%d]: %d\n", q, shiftyTemp);
#endif
    }

#ifdef MASTER

    // we send out to all three supported i2c slave devices
    // keeps the firmware simple :)

    if (notShiftyTemp != lastValue[q])
    {
#ifdef DEBUG
      Serial.printf("i2c Master[%d]: %d\n", q, notShiftyTemp);
#endif

      // for 4 output devices
      port = q % 4;
      device = q / 4;

      // TXo
      sendi2c(0x60, device, 0x11, port, notShiftyTemp);

      // ER-301
      sendi2c(0x31, 0, 0x11, q, notShiftyTemp);

      // ANSIBLE
      sendi2c(0x20, device << 1, 0x06, port, notShiftyTemp);

      lastValue[q] = notShiftyTemp;
    }

#endif
  }

 if (sendNote == noteOnMode)
  {
    if (noteChannel > 16)
    {
#ifdef DEBUG
      Serial.printf("Sending USB MIDI note on channel: %d note %d vel: %d\n", noteChannel - 16, note, noteVelocity);
#endif
      usbMIDI.sendNoteOn(note, noteVelocity, noteChannel - 16);
    } else {
#ifdef DEBUG
      Serial.printf("Sending MIDI note on channel: %d note %d vel: %d\n", noteChannel, note, noteVelocity);
#endif
      MIDI.sendNoteOn(note, noteVelocity, noteChannel);
    }
  }
  else if (sendNote == noteOffMode)
  {
    if (noteChannel > 16)
    {
#ifdef DEBUG
      Serial.printf("Sending USB MIDI note off channel: %d note %d\n", noteChannel - 16, note);
#endif
      usbMIDI.sendNoteOff(note, 0, noteChannel - 16);
    } else {
#ifdef DEBUG
      Serial.printf("Sending MIDI note channel: %d note %d\n", noteChannel, note);
#endif
      MIDI.sendNoteOff(note, 0, noteChannel);
    }
  }
  sendNote = 0;
}

#ifdef MASTER

/*
 * Sends an i2c command out to a slave when running in master mode
 */
void sendi2c(uint8_t model, uint8_t deviceIndex, uint8_t cmd, uint8_t devicePort, int value)
{

  valueTemp = (uint16_t)value;
  messageBuffer[2] = valueTemp >> 8;
  messageBuffer[3] = valueTemp & 0xff;

#ifdef V125
  Wire1.beginTransmission(model + deviceIndex);
  messageBuffer[0] = cmd;
  messageBuffer[1] = (uint8_t)devicePort;
  Wire1.write(messageBuffer, 4);
  Wire1.endTransmission();
#else
  Wire.beginTransmission(model + deviceIndex);
  messageBuffer[0] = cmd;
  messageBuffer[1] = (uint8_t)devicePort;
  Wire.write(messageBuffer, 4);
  Wire.endTransmission();
#endif
}

void sendJFNote(uint8_t voice, int pitch, int velocity)
{
  if (jfMode == 0)
  {
    messageBuffer[0] = 6;
    messageBuffer[1] = 1;
#ifdef V125
    Wire1.beginTransmission(0x70);
    Wire1.write(messageBuffer, 2);
    Wire1.endTransmission();
#else
    Wire.beginTransmission(0x70);
    Wire.write(messageBuffer, 2);
    Wire.endTransmission();
#endif
    jfMode = 1;
  }
   
  messageBuffer[0] = 0x8;
  messageBuffer[1] = voice;
  pitch -= 60;
  int pitchV = pitch * 136 + pitch / 2;
  messageBuffer[2] = pitchV >> 8;
  messageBuffer[3] = pitchV & 0xff;
  if (velocity) velocity = 8000;
  messageBuffer[4] = velocity >> 8;
  messageBuffer[5] = velocity & 0xff;

#ifdef V125
  Wire1.beginTransmission(0x70);
  Wire1.write(messageBuffer, 6);
  Wire1.endTransmission();
#else
  Wire.beginTransmission(0x70);
  Wire.write(messageBuffer, 6);
  Wire.endTransmission();
#endif
}

void sendER301Note(uint8_t voice, int pitch, int velocity)
{
  pitch -= 60;
  int pitchV = pitch * 136 + pitch / 2;
  sendi2c(0x31, 0, 0x11, voice + 16, pitchV);
  sendi2c(0x31, 0, 0x11, voice + 16 + POLYPHONY, velocity);
  sendi2c(0x31, 0, 0, voice + 16, velocity == 0 ? 0 : 1);
#ifdef DEBUG
  Serial.printf("Sent MIDI note to ER-301, voice: %d note: %d vel: %d\n", voice, pitch, velocity);
#endif
}

#else

/*
 * The function that responds to a command from i2c.
 * In the first version, this simply sets the port to be read from.
 */
void i2cWrite(size_t len)
{

#ifdef DEBUG
  Serial.printf("i2c Write (%d)\n", len);
#endif

  // parse the response
  TxResponse response = TxHelper::Parse(len);

  // true command our setting of the input for a read?
  if (len == 1)
  {

    // use a helper to decode the command
    TxIO io = TxHelper::DecodeIO(response.Command);

#ifdef DEBUG
    Serial.printf("Port: %d; Mode: %d [%d]\n", io.Port, io.Mode, response.Command);
#endif

    // this is the single byte that sets the active input
    activeInput = io.Port;
    activeMode = io.Mode;
  }
  else
  {
    // act on the command
    actOnCommand(response.Command, response.Output, response.Value, response.Raw);
  }
}

/*
 * The function that responds to read requests over i2c.
 * This uses the port from the write request to determine which slider to send.
 */
void i2cReadRequest()
{

#ifdef DEBUG
  Serial.print("i2c Read\n");
#endif

  // get and cast the value
  uint16_t shiftReady = 0;
  switch (activeMode)
  {
  case 1:
    shiftReady = (uint16_t)currentValue[activeInput];
    break;
  case 2:
    shiftReady = (uint16_t)currentValue[activeInput];
    break;
  case ccMode:
    if (activeChannel < channelCount && activeCC < maxCCCount)
      shiftReady = lastCCValue[activeChannel][activeCC];
    break;
  default:
    shiftReady = (uint16_t)currentValue[activeInput];
    break;
  }

#ifdef DEBUG
  Serial.printf("delivering: %d; value: %d [%d]\n", activeInput, currentValue[activeInput], shiftReady);
#endif

// send the puppy as a pair of bytes
#ifdef V125
  Wire1.write(shiftReady >> 8);
  Wire1.write(shiftReady & 255);
#else
  Wire.write(shiftReady >> 8);
  Wire.write(shiftReady & 255);
#endif
}

/*
 * Future function if we add more i2c capabilities beyond reading values.
 */
void actOnCommand(byte cmd, byte out, int value, byte *raw)
{
  Serial.println("actOnCommand");
  if (cmd == getMidiCCCommand)
  {
#ifdef DEBUG
    Serial.printf("Received FB.CC channel: %d CC: %d\n", out, raw[1]);
#endif
    activeMode = ccMode;
    activeChannel = out;
    activeCC = raw[1];
  }
  else if (cmd == sendMidiNoteOn)
  {
#ifdef DEBUG
    Serial.printf("Received FB.ON channel: %d note: %d vel: %d\n", out, raw[1], raw[2]);
#endif
    if (out > 0 && out < 33) {
      noteChannel = out;
      sendNote = noteOnMode;
    }
    note = raw[1];
    noteVelocity = raw[2];
  }
  else if (cmd == sendMidiNoteOff)
  {
#ifdef DEBUG
    Serial.printf("Received FB.OFF channel: %d note: %d\n", out, raw[1]);
#endif
    if (out > 0 && out < 33) {
      noteChannel = out;
      sendNote = noteOffMode;
   }
   note = raw[1];
  }
}

#endif
