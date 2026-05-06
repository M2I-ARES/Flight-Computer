// Hyperion FC-Side LoRa Bridge - Feather 32u4 RFM95
// Full duplex byte-at-a-time bridge: Teensy Serial3 <-> LoRa

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   8
#define RFM95_INT  7
#define RFM95_RST  4
#define RF95_FREQ  915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Accumulate bytes from Teensy until we have a complete packet
// then fire it over LoRa in one shot
#define MAX_PKT 64
uint8_t teensyBuf[MAX_PKT];
uint8_t teensyLen = 0;
unsigned long lastTeensyByte = 0;
const unsigned long PKT_TIMEOUT_MS = 20; // flush if no new byte for 20ms

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial1.begin(115200); // UART to Teensy Serial3

  // Hardware reset radio
  digitalWrite(RFM95_RST, LOW);  delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  while (!rf95.init()) { while (1); }
  if (!rf95.setFrequency(RF95_FREQ)) { while (1); }
  rf95.setTxPower(23, false);
}

void flushToLoRa() {
  if (teensyLen > 0) {
    rf95.send(teensyBuf, teensyLen);
    rf95.waitPacketSent();
    teensyLen = 0;
  }
}

void loop() {
  // ── LoRa → Teensy ──────────────────────────────────────────────────
  // Forward immediately, one byte at a time — no buffering, no delay
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      Serial1.write(buf, len);
    }
  }

  // ── Teensy → LoRa ──────────────────────────────────────────────────
  // Accumulate bytes into a buffer, flush when gap detected (packet done)
  while (Serial1.available()) {
    teensyBuf[teensyLen++] = Serial1.read();
    lastTeensyByte = millis();
    if (teensyLen >= MAX_PKT) {
      flushToLoRa(); // buffer full, send now
    }
  }

  // If bytes have been sitting for PKT_TIMEOUT_MS with nothing new, send
  if (teensyLen > 0 && (millis() - lastTeensyByte) >= PKT_TIMEOUT_MS) {
    flushToLoRa();
  }
}
