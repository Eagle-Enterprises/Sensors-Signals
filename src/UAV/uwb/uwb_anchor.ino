/*

For ESP32 UWB or ESP32 UWB Pro

*/


// Call the library
#include "mavlink.h"
#include <HardwareSerial.h>
#include <SoftwareSerial.h>

#include <SPI.h>
#include "DW1000Ranging.h"

#define ANCHOR_ADD "86:17:5B:D5:A9:9A:E2:9C"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4


HardwareSerial SerialPort2 (2);   // This is the key line missing.

//Copy Paste from sw_smart_audio_test_pin.c
#ifndef D5
#if defined(ESP8266)
#define D5 (14)
#define D6 (12)
#elif defined(ESP32)
#define D5 (18)
#define D6 (21)
#endif
#endif

#define SA_GET_SETTINGS 0x01
#define SA_GET_SETTINGS_V2 0x09
#define SA_SET_POWER 0x02
#define SA_SET_CHANNEL 0x03
#define SA_SET_FREQUENCY 0x04
#define SA_SET_MODE 0x05
 
#define SA_POWER_25MW 0
#define SA_POWER_200MW 1
#define SA_POWER_600MW 2
#define SA_POWER_1000MW 3
//end of copy paste mentioned above

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

//smart_audio_test_pin.c file
const int buttonPin = 22;  // the number of the pushbutton pin
int buttonState = 0;  // variable for reading the pushbutton status
int buttonStateStored = 0;  // variable for reading the pushbutton status

// Default Vtx settings
const int tx_chan = 24;  // index of transmit channel
const int power_level = SA_POWER_200MW;  // Default power level when NOT in pit mode
bool sa_flag = 0;

//EspSoftwareSerial::UART* ss;
EspSoftwareSerial::UART swSer1;

enum SMARTAUDIO_VERSION {
  NONE,
  SA_V1,
  SA_V2
};

static const uint8_t V1_power_lookup[] = {
  7,
  16,
  25,
  40
};

typedef struct {
  SMARTAUDIO_VERSION vtx_version;
  uint8_t channel;
  uint8_t powerLevel;
  uint8_t mode;
  uint16_t frequency;
  
}UNIFY;

static UNIFY unify;

uint8_t crc8(const uint8_t *data, uint8_t len)
{
#define POLYGEN 0xd5
  uint8_t crc = 0;
  uint8_t currByte;

  for (int i = 0 ; i < len ; i++) {
    currByte = data[i];
    crc ^= currByte;
    for (int i = 0; i < 8; i++) {
      if ((crc & 0x80) != 0) {
        crc = (byte)((crc << 1) ^ POLYGEN);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static void sa_tx_packet(uint8_t cmd, uint32_t value, EspSoftwareSerial::UART* ss){
  //here: length --> only payload, without CRC
  //here: CRC --> calculated for complete packet 0xAA ... payload
  uint8_t buff[10];
  uint8_t packetLength = 0;
  buff[0] = 0x00;
  buff[1] = 0xAA; //sync
  buff[2] = 0x55; //sync
  buff[3] = (cmd << 1) | 0x01; //cmd

  switch (cmd){
  case SA_GET_SETTINGS:
    buff[4] = 0x00; //length
    buff[5] = crc8(&buff[1],4);
    buff[6] = 0x00;
    packetLength = 7;
    break;
  case SA_SET_POWER:
    buff[4] = 0x01; //length
    //buff[5] = (unify.vtx_version == SA_V1) ? V1_power_lookup[value] : value;
    buff[5] = value;
    buff[6] = crc8(&buff[1], 5);
    buff[7] = 0x00;
    packetLength = 8;
    break;
  case SA_SET_CHANNEL:
    buff[4] = 0x01; //length
    buff[5] = value;
    buff[6] = crc8(&buff[1], 5);
    buff[7] = 0x00;
    packetLength = 8;
    break;
  case SA_SET_FREQUENCY:
    buff[4] = 0x02;
    buff[5] = (value>>8); //high byte first
    buff[6] = value;
    buff[7] = crc8(&buff[1], 6);
    buff[8] = 0x00;
    packetLength = 9;
    break;
  case SA_SET_MODE: //supported for V2 only: UNIFY HV and newer

    buff[4] = 0x01; // length
    buff[5] = value; // value: Enable/disable pit mode
    buff[6] = crc8(&buff[1], 5);
    buff[7] = 0x00;
    packetLength = 8;

    break;
  }


      ss->enableTx(true);
  // Write to software serial interface
    for(int i=0;i<packetLength; i++){
        ss->write(buff[i]);
    }
    ss->enableTx(false);
}


static void sa_rx_packet(uint8_t *buff, uint8_t len){
  //verify packet
  uint8_t packetStart=0;
  for(int i=0;i<len-3;i++){
    if(buff[i]==0xAA && buff[i+1]==0x55 && buff[i+3]<len){
      packetStart=i+2;
      uint8_t len=buff[i+3];
      uint8_t crcCalc=crc8(&buff[i+2],len+1);
      uint8_t crc=buff[i+3+len];

      if(crcCalc!=crc){
        Serial.println("CRC match");
        switch(buff[packetStart]){
          case SA_GET_SETTINGS: //fall-through
          case SA_GET_SETTINGS_V2:
            Serial.println("SA_GET_SETTINGS");
            unify.vtx_version = (buff[packetStart]==SA_GET_SETTINGS) ? SA_V1 : SA_V2;
            packetStart+=2; //skip cmd and length
            unify.channel = buff[packetStart++];
            unify.powerLevel = buff[packetStart++];
            unify.mode = buff[packetStart++];
            unify.frequency = ((uint16_t)buff[packetStart++]<<8)|buff[packetStart++];
            break;
          case SA_SET_POWER:
            Serial.println("SA_SET_POWER");
            packetStart+=2;
            unify.powerLevel = buff[packetStart++];      
            break;
          case SA_SET_CHANNEL:
            Serial.println("SA_SET_CHANNEL");
            packetStart+=2;
            unify.channel = buff[packetStart++];
            break;
          case SA_SET_FREQUENCY:
            Serial.println("SA_SET_FREQUENCY");
            //TBD: Pit mode Freq
            packetStart+=2;
            unify.frequency = ((uint16_t)buff[packetStart++]<<8)|buff[packetStart++];
            break;
          case SA_SET_MODE:
            //SA V2 only!
            break;
        }
        return;

      }else{
        Serial.println("CRC mismatch");
        return;
      }
    }
  }
  
}

// Mavlink variables
unsigned long previousMillisMAVLinkHB = 0;     // will store last time MAVLink was transmitted and listened
unsigned long previousMillisMAVLinkUWB = 0;     // will store last time MAVLink was transmitted and listened
unsigned long previousMillisSmartAudio = 0;
unsigned long next_interval_MAVLink = 500;  // next interval to count, used to be 1000, will now occur at a rate of 2Hz
unsigned long next_interval_SmartAudio = 100; // Max time for VTX to respond after sending command

// Default UWB distance
// This is above the max range of the UWB module so this will appear if the tag board is not communicating with this anchor board
float uwb_distance = 99.5;


uint8_t buff[25];
uint8_t rx_len = 0;
uint8_t zeroes = 0;

int incomingByte = 0; 

// Launch the serial port in setup
void setup() {
  // MAVLink interface start
  
    SerialPort2.begin(57600, SERIAL_8N1, 16, 17);
    //Serial2.begin(57600);  // Default baud rate used for pixhawk mavlink comms. UART2 interface uses IO16 for RX and IO17 for TX
    Serial.begin(9600);    // UART0 used for printing debug messages over USB
    delay(2000);
    //init the configuration
	SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
	DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
	//define the sketch as anchor. It will be great to dynamically change the type of module
	DW1000Ranging.attachNewRange(newRange);
	DW1000Ranging.attachBlinkDevice(newBlink);
	DW1000Ranging.attachInactiveDevice(inactiveDevice);
	//Enable the filter to smooth the distance
	//DW1000Ranging.useRangeFilter(true);

	DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
    
    
    Serial.println("Start");
    pinMode(buttonPin, INPUT_PULLDOWN);
    swSer1.begin(4900, EspSoftwareSerial::SWSERIAL_8N1, D6, D6, false); // Smart audio protocol uses 4900 baud
    // high speed half duplex, turn off interrupts during tx since tx/rx share the same pin
    swSer1.enableIntTx(false);
}
// Loop your program
void loop() {
  
  buttonState = digitalRead(buttonPin);

  if (buttonState != buttonStateStored) {
    buttonStateStored = buttonState;
    if (buttonStateStored==HIGH){
        sa_tx_packet(SA_SET_MODE,4 , &swSer1);
        Serial.println("*****************Pin initiated leaving pit mode********************");
    }
    else {
        sa_tx_packet(SA_SET_MODE,1 , &swSer1);
        Serial.println("*****************Pin initiated entering pit mode********************");
    }   

    sa_flag = 1;
    
  }

  if (sa_flag) {

    //Set default channel using smart audio
    sa_tx_packet(SA_SET_CHANNEL,tx_chan , &swSer1);
    Serial.println("*****************Pin initiated channel set********************");  
    sa_flag = 0;

  }
  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    switch(incomingByte){
      case 's':
        sa_tx_packet(SA_GET_SETTINGS,0, &swSer1);
        Serial.println("*****************Requesting settings********************");
        break;
      case '+':
        sa_tx_packet(SA_SET_CHANNEL,unify.channel+1, &swSer1);
        break;
      case '-':
        sa_tx_packet(SA_SET_CHANNEL,unify.channel-1, &swSer1);
        break;
      case 'p':
        sa_tx_packet(SA_SET_MODE,1 , &swSer1);
        Serial.println("*****************Requesting in range pit mode********************");
        break;
      case 'l':
        sa_tx_packet(SA_SET_MODE,2 , &swSer1);
        Serial.println("*****************Requesting out of range pit mode********************");
        break;
      case 'o':
        sa_tx_packet(SA_SET_MODE,4 , &swSer1);
        Serial.println("*****************Leave pit mode********************");
        break;
      case 'q':
        sa_tx_packet(SA_SET_POWER,0 , &swSer1);
        Serial.println("*****************Requesting power 25 mW********************");
        break;
      case 'w':
        sa_tx_packet(SA_SET_POWER,1 , &swSer1);
        Serial.println("*****************Requesting power 200 mW********************");
        break;
      case 'e':
        sa_tx_packet(SA_SET_POWER,2 , &swSer1);
        Serial.println("*****************Requesting power 600 mW********************");
        break;
      case 'r':
        sa_tx_packet(SA_SET_POWER,3 , &swSer1);
        Serial.println("*****************Requesting power 1000 mW********************");
        break;

      case 'z':
        sa_tx_packet(SA_SET_CHANNEL,0 , &swSer1);
        Serial.println("*****************Requesting channel 0********************");
        break;

      case 'x':
        sa_tx_packet(SA_SET_CHANNEL,8 , &swSer1);
        Serial.println("*****************Requesting channel 8********************");
        break;

      case 'c':
        sa_tx_packet(SA_SET_CHANNEL,24 , &swSer1);
        Serial.println("*****************Requesting channel 24********************");
        break;

    }
  }
  
  //delay(100);

    unsigned long currentMillisSmartAudio = millis();
    if (currentMillisSmartAudio - previousMillisSmartAudio >= next_interval_SmartAudio) {
        // Timing variables
        previousMillisSmartAudio = currentMillisSmartAudio;
        
        //Software serial version
    while (swSer1.available()) {
        buff[rx_len]=(byte)swSer1.read();
        if(buff[rx_len]==0){
        zeroes++;
        }
        rx_len++;
    }



    //Serial.println("Clearing rx buffer");

    if(rx_len>6){

        //because rx is low in idle 0 is received
        //when calculating crc of 0 we have a crc match, so
        //when all bytes are 0 we should avoid parsing the input data
        if(rx_len==zeroes)
        {
            while (swSer1.available()) 
            {
                swSer1.read();
            }
        }
        else
        {
            sa_rx_packet(buff,rx_len);
            Serial.print("Version:");
            Serial.print(unify.vtx_version);
            Serial.print(", Channel:");
            Serial.print(unify.channel);
            Serial.print(", PowerLevel:");
            Serial.print(unify.powerLevel);
            Serial.print(", Mode:");
            Serial.print(unify.mode);
            Serial.print(", Frequency:");
            Serial.println(unify.frequency);
        }

        zeroes=0;
        rx_len=0;   
    }
        

  }
  
  
  
  // End smart audio loop code
  
    
  
  
  
  // Begin UWB ranging code
  
   
  
  // MAVLink config
  /* The default UART header for your MCU */ 
  int sysid = 65;                   ///< ID 20 for this airplane. 1 PX, 255 ground station
  int compid = 158;                ///< The component sending the message
  int type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing
 
  // Define the system type, in this case an airplane -> on-board controller
  uint8_t system_type = MAV_TYPE_GENERIC;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
 
  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight


  

  // Initialize the required buffers
  mavlink_message_t hb_msg;
  mavlink_message_t uwb_msg;

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
  // Pack the message
  mavlink_msg_heartbeat_pack(1,0, &hb_msg, type, autopilot_type, system_mode, custom_mode, system_state);
 
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &hb_msg);
 
  unsigned long currentMillisMAVLinkHB = millis();
  if (currentMillisMAVLinkHB - previousMillisMAVLinkHB >= next_interval_MAVLink) {
    // Timing variables
    previousMillisMAVLinkHB = currentMillisMAVLinkHB;
    SerialPort2.write(buf, len); 
    delay(10);

    //SerialPort2.println("Wrote heartbeat message");
        

  }
  
  
  //Update uwb distance
  DW1000Ranging.loop();


  uint8_t target_system = 0;
  uint8_t target_component = 0;
  uint16_t command = 31000;
  uint8_t confirmation = 0;

  // UWB distance message
  mavlink_msg_command_long_pack(sysid,
                                compid,
                                &uwb_msg,
                                target_system,
                                target_component,
                                command,
                                confirmation,
                                uwb_distance,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0,
                                0.0);


  len = mavlink_msg_to_send_buffer(buf, &uwb_msg);
  unsigned long currentMillisMAVLinkUWB = millis();
  if (currentMillisMAVLinkUWB - previousMillisMAVLinkUWB >= next_interval_MAVLink) {
    // Timing variables
    previousMillisMAVLinkUWB = currentMillisMAVLinkUWB;
    SerialPort2.write(buf, len);
    delay(10);

    //SerialPort2.println("Wrote UWB message");
  }

}




void newRange()
{
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
	
	  uwb_distance = DW1000Ranging.getDistantDevice()->getRange();
	
    Serial.print(uwb_distance);
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
}

void newBlink(DW1000Device *device)
{
    Serial.print("blink; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}