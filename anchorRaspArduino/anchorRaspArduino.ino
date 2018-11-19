#define DEBUG true

#include <SPI.h>

#include <DW1000.h>

#include "debug.h"
#include "def.h"
#include "i2c.h"
#include "arduino.h"

#define PIN_IRQ  2
#define PIN_RST  9
#define PIN_SS  SS

/* Edit anchorId */
// Each anchor must have a unique anchor ID (do not use ID 0)
const uint16_t anchorId = 1;
// UWB anchors and tags must have the same network ID
const uint16_t networkId = 0xDECA;
static uint8_t eui[] = {'A', 'C', 'K', 'D', 'A', 'T', 'R', 'X'};
// Sender of the last received frame in the S/W buffer
uint16_t sender;
// Counter part tag during two way ranging (PING, PONNG, RANGE, RANGEREPORT)
uint16_t tagCounterPart = ID_NONE;

// Current state of a UWB anchor state machine
char state = STATE_IDLE;

// UWB anchors and tags must have the same replay_delay
DW1000Time reply_delay = DW1000Time(REPLY_DELAY_MS, DW1000Time::MILLISECONDS);
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timeRangeReceived;

// Last time that loop() is called
unsigned long curMillis;
// Last time that a frame is PUSHED INTO THE AIR FROM THE S/W BUFFER
unsigned long lastSent;
// Last time that a UWB device sends or receives a frame
// i.e., meaningful DWM1000 activity
unsigned long lastActivity;
unsigned long lastStateChange;

byte txBuffer[FRAME_LEN];
byte rxBuffer[FRAME_LEN];

// Set to true if a frame is pushed into the air and SPI tx interrupt is triggered
boolean sentFrame = false;
// Set to true if a frame is received and SPI rx interrupt is triggered
// Not yet stored into the S/W buffer
boolean receivedFrame = false;

// getValue for test
char deviceIdBuffer[40];
char deviceIdentifierBuffer[40];
char extendedUniqueIdentifierBuffer[40];
char NetworkIdAndShortAddressBuffer[40];
char deviceModeBuffer[40];

void updateState(int nextState) {
  state = nextState;
  lastStateChange = millis();
}

void noteActivity() {
  lastActivity = millis();
}

/*************************************
 * Arduino (master) - DW1000 (slave) *
 *************************************/
void spiReceiveEvent() {
  receivedFrame = true;
}

void spiSendEvent() {
  sentFrame = true;
}

void initDW1000Receiver() {
  DW1000.newReceive();
  // DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
  noteActivity();
}

void setupDW1000() {
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  // DW1000.setNetworkId(networkId);
  // DW1000.setDeviceAddress(anchorId);
  DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_LOWPOWER);  
  DW1000.setChannel(DW1000.CHANNEL_1);
  // DW1000.setEUI(eui);

  DW1000.setPreambleCode(DW1000.PREAMBLE_CODE_16MHZ_2);
  DW1000.commitConfiguration();

  // test
  PRINTLN(F("start test"));
  DW1000.getPrintableDeviceIdentifier(deviceIdBuffer);
  PRINTLN(deviceIdBuffer);
  DW1000.getPrintableExtendedUniqueIdentifier(extendedUniqueIdentifierBuffer);
  PRINTLN(extendedUniqueIdentifierBuffer);
  DW1000.getPrintableNetworkIdAndShortAddress(NetworkIdAndShortAddressBuffer);
  PRINTLN(NetworkIdAndShortAddressBuffer);
  DW1000.getPrintableDeviceMode(deviceModeBuffer);
  PRINTLN(deviceModeBuffer);

  DW1000.attachSentHandler(spiSendEvent);
  DW1000.attachReceivedHandler(spiReceiveEvent);

  initDW1000Receiver();
}

void prepareTx() {
  DW1000.newTransmit();
  // DW1000.setDefaults();
}

void startTx() {
  DW1000.setData(txBuffer, FRAME_LEN);
  DW1000.startTransmit();
  // timeout will be asserted after tx interrupt
  lastSent = 0;
}
void transmitPing() {
  prepareTx();
  txBuffer[9] = 0x80;
  // SET_SRC(txBuffer, anchorId, ADDR_SIZE);
  startTx();
}

void transmitPong() {
  prepareTx();
  txBuffer[9] = 0x80;
  SET_SRC(txBuffer, anchorId, ADDR_SIZE);
  startTx();
}

void transmitPollAck() {
  prepareTx();
  txBuffer[9] = TOA_POLLACK;
  // SET_SRC(txBuffer, anchorId, ADDR_SIZE);
  // SET_DST(txBuffer, tagCounterPart, ADDR_SIZE);
  txBuffer[DESTADD] = rxBuffer[SOURADD];
  txBuffer[DESTADD+1] = rxBuffer[SOURADD+1];
  txBuffer[SOURADD] = (uint8_t)(anchorId);
  txBuffer[SOURADD+1] = (uint8_t)(anchorId>>8);

  DW1000.setDelay(reply_delay);
  startTx();
}

void transmitRangeReport() {
  prepareTx();
  txBuffer[9] = TOA_RANGEACK;
  // SET_SRC(txBuffer, anchorId, ADDR_SIZE);
  // SET_DST(txBuffer, tagCounterPart, ADDR_SIZE);

  txBuffer[DESTADD] = rxBuffer[SOURADD];
  txBuffer[DESTADD+1] = rxBuffer[SOURADD+1];
  txBuffer[SOURADD] = (uint8_t)(anchorId);
  txBuffer[SOURADD+1] = (uint8_t)(anchorId>>8);

  timePollReceived.getTimestamp(txBuffer + 5);
  timePollAckSent.getTimestamp(txBuffer + 10);
  timeRangeReceived.getTimestamp(txBuffer + 15);
  startTx();
}

/********
 * Main *
 ********/
void setup() {
  #if DEBUG
  Serial.begin(115200);
  #endif // DEBUG
  
  // setupI2C();
  setupDW1000();

  txBuffer[0] = 0x41;
  txBuffer[1] = 0x88;
  txBuffer[3] = 0xCA;
  txBuffer[4] = 0xDE;
  txBuffer[5] = 0xFF;
  txBuffer[6] = 0xFF;

  PRINTLN(F("Setup finished"));
  PRINTLN(F("=============="));
  randomSeed(analogRead(0));
}

void loop() {
  curMillis = millis();
  // Arduino didn't capture SPI tx/rx interrupts for more than RESET_TIMEOUT_MS
  if (!sentFrame && !receivedFrame && curMillis - lastActivity > RESET_TIMEOUT_MS) {
    // PRINTLN(F("Seems transceiver not working. Re-init it."));
    transmitPing();
    PRINTLN(F("Sent Ping"));
    return;
  }

  // SPI tx interrupt is captured
  // Update some time variables, state
  // Extract DW1000 high-precision time value if needed
  if (sentFrame) {
    PRINTLN(F("Sent something"));
    sentFrame = false;
    noteActivity();
    lastSent = lastActivity;
  }
}
