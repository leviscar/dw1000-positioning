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
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();
  noteActivity();
}

void setupDW1000() {
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setNetworkId(networkId);
  DW1000.setDeviceAddress(anchorId);
  DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
  DW1000.setChannel(DW1000.CHANNEL_1);
  DW1000.setEUI(eui);
  // DW1000.setFrameFilter(true);
  // DW1000.setFrameFilterAllowData(true);
  // DW1000.setFrameFilterAllowAcknowledgement(true);
  DW1000.setPreambleCode(DW1000.PREAMBLE_CODE_16MHZ_2);
  DW1000.enableLedBlinking();
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
  DW1000.setDefaults();
}

void startTx() {
  DW1000.setData(txBuffer, FRAME_LEN);
  DW1000.startTransmit();
  // timeout will be asserted after tx interrupt
  lastSent = 0;
}

void transmitPong() {
  prepareTx();
  txBuffer[9] = 0x80;
  SET_SRC(txBuffer, anchorId, ADDR_SIZE);
  SET_DST(txBuffer, sender, ADDR_SIZE);
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

  PRINTLN(F("Setup finished"));
  PRINTLN(F("=============="));
  randomSeed(analogRead(0));
}

void loop() {
  curMillis = millis();
  // Safety watchdog to avoid stuck in PENDING_PONG state
  // Sometimes SPI tx interrupt may not be captured by Arduino
  if (state == STATE_PENDING_PONG
      && curMillis - lastStateChange > PENDING_PONG_TIMEOUT_MS) {
    PRINTLN(F("Seems Pending Pong lost. Return to IDLE"));
    updateState(STATE_IDLE);
  }
  // Safety watchdog to avoid stuck in RANGE state
  // 1. If SPI tx interrupt is captured (confirmed that POLLACK is sent)
  // 2. If SPI tx interrupt is not captured for some reasons
  if (state == STATE_RANGE
      && ((lastSent && curMillis - lastSent > RANGE_TIMEOUT_MS)
          || curMillis - lastStateChange > 2 * RANGE_TIMEOUT_MS)) {
    /*
     * Check RANGE message timeout when state is waiting for RANGE message
     */
    PRINTLN(F("RANGE timeout. Return to IDLE"));
    updateState(STATE_IDLE);
    return;
  }
  // Arduino didn't capture SPI tx/rx interrupts for more than RESET_TIMEOUT_MS
  if (!sentFrame && !receivedFrame && curMillis - lastActivity > RESET_TIMEOUT_MS) {
    PRINTLN(F("Seems transceiver not working. Re-init it."));
    transmitPong();
    PRINTLN(F("Sent pong"));
    initDW1000Receiver();
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

    if (state == STATE_PENDING_PONG && txBuffer[0] == FTYPE_PONG) {
      PRINTLN(F("  Pending PONG sent. Return to IDLE"));
      updateState(STATE_IDLE);
      return;
    }

    if (txBuffer[9] == TOA_POLLACK) {
      PRINTLN(F("  POLLACK sent. Getting timestamp..."));
      DW1000.getTransmitTimestamp(timePollAckSent);
    }

    if (txBuffer[9] == TOA_RANGEACK) {
      PRINTLN(F("  RANGEREPORT sent"));
    }
  }

  // SPI rx interrupt is captured
  //  Update some time variables, state
  // Extract DW1000 high-precision time value if needed
  if (receivedFrame) {
    PRINTLN(F("Received something"));
    receivedFrame = false;
    noteActivity();
    DW1000.getData(rxBuffer, FRAME_LEN);
    GET_SRC(rxBuffer, sender, ADDR_SIZE);

    PRINTLN(rxBuffer[0]);
    PRINTLN(rxBuffer[9]);

    if (state == STATE_IDLE) {
      PRINTLN(F("  State: IDLE"));
      if (rxBuffer[9] == TOA_POLL) {
        PRINTLN(F("    Received POLL"));
        // if (!DOES_DST_MATCH(rxBuffer, anchorId, ADDR_SIZE)) {
        //   PRINTLN(F("      Not for me"));
        //   return;
        // }
        PRINTLN(F("      Reply with POLLACK"));
        DW1000.getReceiveTimestamp(timePollReceived);
        tagCounterPart = sender;
        transmitPollAck();
        updateState(STATE_RANGE);
        return;
      }
    }

    if (state == STATE_RANGE) {
      PRINTLN(F("  State: RANGE"));
      if (rxBuffer[9] != TOA_RANGE) {
        PRINTLN(F("    Not RANGE"));
        return;
      }
      // if (!DOES_SRC_MATCH(rxBuffer, tagCounterPart, ADDR_SIZE)) {
      //   PRINTLN(F("    Not from counter part"));
      //   return;
      // }
      PRINTLN(F("    Sending RANGEREPORT..."));
      DW1000.getReceiveTimestamp(timeRangeReceived);
      transmitRangeReport();
      updateState(STATE_IDLE);
      return;
    }
  }
}
