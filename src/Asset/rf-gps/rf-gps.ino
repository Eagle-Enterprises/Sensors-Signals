// Include Libraries
#include <SPI.h>
#include <RH_RF69.h>

// #define DEBUG //Only meant to be run when connected to a PC + outputting to a Serial Monitor

// Define Operating Frequency: 915 for mission operations
#define RF69_FREQ 915.0

// Define Board Pins
// Current code assumes use of an ESP8266; consult example code "RadioHead69_RawDemo_TX/RX if different" 
#if defined(ESP8266)
  #define RFM69_CS    15    // Pin D8 on board == GPIO16
  #define RFM69_INT   5     // Pin D1
  #define RFM69_RST   16    // Pin D0
  #define LED         2     // Pin D4, pre-connected to on-board LED
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

void setup() {
  Serial.begin(115200);

  #ifdef DEBUG
    while (!Serial) delay(1); // Wait for Serial Console
  #endif

  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 RX Test!");
  Serial.println();

  resetRF69();
  
  Serial.println("RFM69 radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

}

void loop() {
  // Main loop: If the radio module is available, construct a message
  if (!rf69.available()) {
    Serial.println("RFM69 radio not available");
    resetRF69();
  }

  // Generate Message to send
  uint8_t buf[22]; // TODO: Should be variable based on number of args given
  uint8_t len = sizeof(buf);



}

void resetRF69() {
  Serial.println("Resetting RFM 69 Radio...");
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
}

void Blink(byte pin, byte delay_ms, byte loops) {
  while (loops--) {
    digitalWrite(pin, HIGH);
    delay(delay_ms);
    digitalWrite(pin, LOW);
    delay(delay_ms);
  }
}





