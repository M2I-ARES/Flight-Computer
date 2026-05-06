// Hyperion Ground Station - Feather 32u4 RFM95
// Pure bridge: USB Serial (Pi) <-> LoRa (FC)
// All protocol logic is handled by Python on the Pi

#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define RF95_FREQ  915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);  // USB to Pi
  while (!Serial) delay(1);

  // Hardware reset
  digitalWrite(RFM95_RST, LOW);  delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  

  while (!rf95.init()) { while (1); }   // halt if radio fails
  if (!rf95.setFrequency(RF95_FREQ)) { while (1); }
  rf95.setTxPower(23, false);
}

void loop() {
  // LoRa → Pi: forward received packet raw bytes over USB
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      Serial.write(buf, len);
    }
  }

  // Pi → LoRa: collect bytes from USB, send over LoRa
  if (Serial.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = 0;
    // small delay to let the full frame arrive before sending
    delay(5);
    while (Serial.available() && len < sizeof(buf)) {
      buf[len++] = Serial.read();
    }
    if (len > 0) {
      rf95.send(buf, len);
      rf95.waitPacketSent();
    }
  }
}
