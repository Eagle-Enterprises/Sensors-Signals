#include <HardwareSerial.h>
#include <SoftwareSerial.h>

//Target board - ESP32 or ESP8266 
#define ESP32


//Using pin D6 for half duplux smart audio

#ifndef D5
#if defined(ESP8266)
#define D5 (14)
#define D6 (12)
#elif defined(ESP32)
#define D5 (18)
#define D6 (23)
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
#define SA_POWER_500MW 2
#define SA_POWER_800MW 3


const int buttonPin = 22;  // the number of the pushbutton pin
int buttonState = 0;  // variable for reading the pushbutton status
int buttonStateStored = 0;  // variable for reading the pushbutton status


const int tx_chan = 24;  // index of transmit channel

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

      //TBD --> Pit mode
      /*
      buffer[4] = 0x01; //length
      buffer[5] = value;
      buffer[6] = crc8(&buffer[1], 5);
      buffer[7] = 0x00;
      packetLength = 8;s
      */
    break;
  }


    ss->enableTx(true);
  // Write to software serial interface
    for(int i=0;i<packetLength; i++){
        ss->write(buff[i]);
    }
    ss->enableTx(false);

//   for(int i=0;i<packetLength; i++){
//     Serial4.write(buff[i]);
//   }
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




void setup()
{

    delay(2000);
  Serial.begin(9600);
  //while (!Serial);             // Leonardo: wait for serial monitor

  Serial.println("Start");
  //Serial4.begin(4900);
  //UCSR0B &= ~(1<<TXEN0);

  pinMode(buttonPin, INPUT);


    swSer1.begin(4900, EspSoftwareSerial::SWSERIAL_8N1, D6, D6, false);
    // high speed half duplex, turn off interrupts during tx
    swSer1.enableIntTx(false);
}

uint8_t buff[25];
uint8_t rx_len = 0;
uint8_t zeroes = 0;

int incomingByte = 0; 
void loop(){


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
       
       //TODO: set channel on same interval as heartbeat
       // sa_tx_packet(SA_SET_CHANNEL,tx_chan , &swSer1);
       // Serial.println("*****************Pin initiated channel set********************");
    }

  
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    //Serial4.begin(4900);
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
        Serial.println("*****************Requesting power 7********************");
        break;
      case 'w':
        sa_tx_packet(SA_SET_POWER,1 , &swSer1);
        Serial.println("*****************Requesting power 16********************");
        break;
      case 'e':
        sa_tx_packet(SA_SET_POWER,2 , &swSer1);
        Serial.println("*****************Requesting power 25********************");
        break;
      case 'r':
        sa_tx_packet(SA_SET_POWER,3 , &swSer1);
        Serial.println("*****************Requesting power 40********************");
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
    //Serial4.end();//clear buffer, otherwise sa_tx_packet is received
    //Serial4.begin(4900);
    //UCSR0B &= ~(1<<TXEN0); //deactivate tx --> rx mode listening for response
  }
  
  delay(100);

  Serial.println("Waiting for rx data");
  
    //Software serial version
  while (swSer1.available()) {
    buff[rx_len]=(byte)swSer1.read();
    if(buff[rx_len]==0){
      zeroes++;
    }
    //Serial.print(buff[rx_len],HEX);
    //Serial.print(",");
    rx_len++;
  }



  //Original hardware version
//   while(Serial4.available()){
//     buff[rx_len]=Serial4.read();
//     if(buff[rx_len]==0){
//       zeroes++;
//     }
//     //Serial.print(buff[rx_len],HEX);
//     //Serial.print(",");
//     rx_len++;
//   }
  //Serial.println();


    Serial.println("Clearing rx buffer");

  if(rx_len>6){

    //because rx is low in idle 0 is received
    //when calculating crc of 0 we have a crc match, so
    //when all bytes are 0 we should avoid parsing the input data
    if(rx_len==zeroes){

    while (swSer1.available()) {
        swSer1.read();
    }


    //   while(Serial4.available()){
    //     Serial4.read();
    //   }
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
