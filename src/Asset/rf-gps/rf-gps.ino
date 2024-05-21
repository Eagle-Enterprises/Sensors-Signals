// Include Libraries
#include <SPI.h>
#include <RH_RF69.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

// Uncomment the below line when connected to a PC + outputting to a Serial Monitor
#define DEBUG

// Define Operating Frequency: 915 for mission operations
#define RF69_FREQ 915.0

// Define Board Pins
// Current code assumes use of an ESP8266; consult example code "RadioHead69_RawDemo_TX/RX if different" 
#if defined(ESP8266)
  #define RFM69_CS    15    // Pin D8 on board == GPIO16
  #define RFM69_INT   5     // Pin D1
  #define RFM69_RST   16    // Pin D0
  #define LED         2     // Pin D4, pre-connected to on-board LED
  #define D2_TX       0
  #define D3_RX       4
#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);
TinyGPS gps;
SoftwareSerial ss_gps(D3_RX, D2_TX);

void setup() {
  Serial.begin(115200);
  ss_gps.begin(9600);

  // #ifdef DEBUG
  //   while (!Serial) delay(1); // Wait for Serial Console
  // #endif

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
  bool newData = false;
  // Main loop: If the radio module is available, construct a message

  for (unsigned long start = millis(); millis() - start < 500;)
  {
    while (ss_gps.available())
    {
      char c = ss_gps.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    int num_satellites = gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites();

    // Generate Message to send
    char buf[37] = "LAT:"; // TODO: Should be variable based on number of args given
    // Expected Message Format:
    // LAT:XX.xxxxxx, LON:XX.xxxxxx, SAT:XX
    uint8_t len = sizeof(buf);

    ftoa(buf+4, flat, 6);
    strcat(buf, ", LON:");
    ftoa(buf+19, flon, 6);
    strcat(buf, ", SAT:");
    itoa(num_satellites, buf+35, 10);

    #ifdef DEBUG
       Serial.println(buf);
    #endif

    rf69.send((uint8_t *)buf, strlen(buf));
    rf69.waitPacketSent();

  }

  #ifdef DEBUG
    unsigned long chars;
    unsigned short sentences, failed;
    gps.stats(&chars, &sentences, &failed);
    Serial.print(" CHARS=");
    Serial.print(chars);
    Serial.print(" SENTENCES=");
    Serial.print(sentences);
    Serial.print(" CSUM ERR=");
    Serial.println(failed);
    if (chars == 0)
      Serial.println("** No characters received from GPS: check wiring **");
  #endif

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

void Blink(byte pin, byte delay_ms, byte loops) 
{
  while (loops--) {
    digitalWrite(pin, HIGH);
    delay(delay_ms);
    digitalWrite(pin, LOW);
    delay(delay_ms);
  }
}

char *ftoa(char *buffer, double d, int precision) 
{
  long wholePart = (long) d;

  // Deposit the whole part of the number.
  itoa(wholePart,buffer,10);

  // Now work on the faction if we need one.
  if (precision > 0) {
    // We do, so locate the end of the string and insert
    // a decimal point.
    char *endOfString = buffer;
    while (*endOfString != '\0') endOfString++;
    *endOfString++ = '.';

    // Now work on the fraction, be sure to turn any negative
    // values positive.
    if (d < 0) {
      d *= -1;
      wholePart *= -1;
    }
    
    double fraction = d - wholePart;
    while (precision > 0) {

      // Multipleby ten and pull out the digit.
      fraction *= 10;
      wholePart = (long) fraction;
      *endOfString++ = '0' + wholePart;

      // Update the fraction and move on to the
      // next digit.
      fraction -= wholePart;
      precision--;
    }

    // Terminate the string.
    *endOfString = '\0';
  }
    return buffer;
}
