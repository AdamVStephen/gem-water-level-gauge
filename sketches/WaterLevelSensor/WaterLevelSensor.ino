#include <Ethernet.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <EthernetClient.h>
#include <Dhcp.h>
#include <Dns.h>

/*
  WaterLevelSensor.ino, Adam Stephen, https://github.com/AdamVStephen

    TODO

  - Make low power
  - Add a configuration parameter for the delay time
  - Make the configuration interface generic
  - Fix the UDP issue (random delays/timeouts)
  - Calibration from DAC to mm
  - Enforce LOLO < LO < HI < HIHI
  -


  Provenance:
  1. Initially derived from WaterLevelSensorUdp by Fredrik Moberg, http://brelovich.com/  (which in turn credits 2.)
  2. Based on Udp sample code created 21 Aug 2010 by Michael Margolis

  Overview

  Inputs :
  - analogue input A0 assumes a pressure level sensor MPX5010DP providing 0-5V.
  - serial port is monitored for configuration and interrogation commands

  Configuration : configuration settings are serial and/or jumper settings.
  Configuration settings via serial are persistent via EEPROM across reset.

  Following EPICS alarm definitions, we map the 0-Full range into 5 divisions.
  The thresholds for these alarm levels can be set by serial

  LOLO   : Very low - refill urgently
  LO     : Low - refill soon
  NORMAL : Mid range
  HI     : Near full
  HIHI   : Very near full

  Output : options to show the water level in the tank via a number of media :

  (a) Serial                  [Implemented : Y]
  (b) Ethernet                [Implemented : Y]
  (c) Wireless                [Implemented : N]
  (d) LED traffic light       [Implemented : N]
  (e) LED matrix level meter  [Implemented : N]
  (f) 7-Segment Display 0-9   [Implemented : N]
  (g) LCD 16x2 display        [Implemented : N]

  The active display modes can be combined, subject to pin limits.   Display selection TBA.

  Status monitoring :

  This sketch will monitor analog input A0 and send UDP packets on the ethernet port
  Indicating the level 0-1024 and status LOW, MED or HIGH, based on the thresholds that
  can be configured through the serial interface. If the level is under the LOW threshold
  the status will be set to ERR to indicate an error, such as the sensor being disconnected
  or broken.

  To configure the thresholds connect to the serial port at 9600 bps 8N1 and issue commands
  according to this syntax:

  To read a threshold
  g level

  where
  - level = [L|l|h|H] for lolo, lo, hi, hihi


  To set a threshold

  s level number

  where
  - level = [L|l|h|H] for lolo, lo, hi, hihi
  - number is an integer 0-1023



*/

#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <Udp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <stdio.h>
#include <EEPROM.h>  //We will read and write config from here

int trace = 0;

// include the library code:
#include <LiquidCrystal.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = { 0x90, 0xA2, 0xDA, 0x00, 0x5C, 0x2C };
byte ip[] = { 192, 168, 0, 210 }; //Set according to your network setup
byte gw[] = { 192, 168, 0, 1 }; //Set according to your network setup
byte nm[] = { 255, 255, 255, 0 }; //Recipient of packets. In this case the broadcast address for the local network

unsigned int localPort = 9876;      // local port to listen on

// the next two variables are set when a packet is received
byte remoteIp[4] = { 192, 168, 0, 255 };
unsigned int remotePort = 9876;

// EthernetUDP instance to send UDP packets
EthernetUDP Udp;


// buffers for receiving and sending data
//char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char ReplyBuffer[21];
//char LcdBufferTopRow[16];
//char LcdBufferBottomRow[16];
char LcdTopRowBuffer[16];
char LcdBottomRowBuffer[16];

// Trigger levels for alarms : by inference tNormal is tLo < level < tHi
// Will be fetched from eeprom
int16_t tLoLo = 100;
int16_t tLo = 200;
int16_t tHi = 900;
int16_t tHiHi = 1000;

int16_t nCurrentLevel = 0;

int32_t delay_time = 0;

//Message prefixes
char msgPrefix[5][7] = { "LOLO ", "LO ", "NORMAL", "HI", "HIHI" };
byte nWarnLevel = 0;
#define LEVEL_LOLO 0
#define LEVEL_LO 1
#define LEVEL_NORMAL 2
#define LEVEL_HI 3
#define LEVEL_HIHI 4

//Character buffer and pointer(s) for Serial command parsing
const int serialBufSize = 20;
char serialBuf [serialBufSize + 1];
unsigned int nSerialBufPos = 0;

// Caveat : EEPROM memory has a specified life of 100k write/erase cycles
//
// SO : only support changing parameters infrequently !
//
// Takes 3.3ms per write()


void writeInt16 (uint16_t nAddress, int16_t nData) {
  //Write 2 bytes to EEPROM, starting at the given address.
  EEPROM.write (nAddress, nData >> 8);
  EEPROM.write (nAddress + 1, nData & 0xFF);
  return;
}

int16_t readInt16 (uint16_t nAddress) {
  // Read 2 bytes from EEPROM, starting at the given address.
  int16_t nMSB = EEPROM.read (nAddress);
  nMSB = nMSB << 8;
  int16_t nLSB = EEPROM.read (nAddress + 1);
  return (nMSB | nLSB);
}

void writeInt32 (uint32_t nAddress, int32_t nData) {
  //Write 4 bytes to EEPROM, starting at the given address.
  Serial.println("writeInt32 +0 +1 +2 +3");
  Serial.println(nAddress);
  Serial.println(nData);
  Serial.println((nData & 0xFF000000)>>24);
  Serial.println((nData & 0x00FF0000)>>16);
  Serial.println((nData & 0x0000FF00)>>8);
  Serial.println(nData & 0x000000FF);
  EEPROM.write (nAddress + 0, ((nData & 0xFF000000) >> 24));
  EEPROM.write (nAddress + 1, ((nData & 0x00FF0000) >> 16));
  EEPROM.write (nAddress + 2, ((nData & 0x0000FF00) >> 8));
  EEPROM.write (nAddress + 3, ((nData & 0x000000FF) >> 0));
  return;
}

int32_t readInt32 (uint32_t nAddress) {
  // Read 4 bytes from EEPROM, starting at the given address.
  int32_t MSB3 = EEPROM.read (nAddress + 0);
  MSB3 = MSB3 << 24;
  int32_t MSB2 = EEPROM.read (nAddress + 1);
  MSB2 = MSB2 << 16;
  int32_t MSB1 = EEPROM.read (nAddress + 2);
  MSB1 = MSB1 << 8;
  int32_t MSB0 = EEPROM.read (nAddress + 3);
  //
  return (MSB3 | MSB2 | MSB1 | MSB0);
}

void uptime() {
  Serial.print("Uptime(ms): ");
  Serial.println(millis());
}

void simWriteInt32 (uint32_t nAddress, int32_t nData) {
  //Write 4 bytes to EEPROM, starting at the given address.
  Serial.println("simWriteInt32");
  int32_t b;
  for (int32_t i = 0; i < 4; i++) {
    b = nData & (0xFF000000 >> (8 * i));
    Serial.println("Values follow : nData; i; b");
    Serial.println(nData);
    Serial.println(i);
    Serial.println(b);
    Serial.println("--");
    sprintf(serialBuf, "0x%x i 0x%x b 0x%x", nData, i, b);
    Serial.println(serialBuf);
    sprintf(serialBuf, "0x%x b 0x%x i 0x%x", nData, b, i);
    Serial.println(serialBuf);
  }
  return;
}

int32_t simReadInt32 (uint32_t nAddress) {
  // Read 4 bytes from EEPROM, starting at the given address.
  int32_t MSB3 = EEPROM.read (nAddress + 0);
  MSB3 = MSB3 << 24;
  int32_t MSB2 = EEPROM.read (nAddress + 1);
  MSB2 = MSB2 << 16;
  int32_t MSB1 = EEPROM.read (nAddress + 2);
  MSB1 = MSB1 << 8;
  int32_t MSB0 = EEPROM.read (nAddress + 3);
  //
  int32_t ret = (MSB3 | MSB2 | MSB1 | MSB0);

  Serial.print("simReadInt32 from address ");
  Serial.println(nAddress);
  sprintf(serialBuf, "3 0x%x", MSB3); Serial.println(serialBuf);
  sprintf(serialBuf, "2 0x%x", MSB2); Serial.println(serialBuf);
  sprintf(serialBuf, "1 0x%x", MSB1); Serial.println(serialBuf);
  sprintf(serialBuf, "0 0x%x", MSB0); Serial.println(serialBuf);
  sprintf(serialBuf, "Z 0x%x", ret); Serial.println(serialBuf);

  sprintf(serialBuf, "M 0x%x", 1000000); Serial.println(serialBuf);

  return;


}


void serialHelp() {
  Serial.println("");
  Serial.println ("*****************************************");
  Serial.println ("* Serial Interface to WaterLevelMonitor *");
  Serial.println ("*                                       *");
  Serial.println ("* Copyright Adam Stephen, Infinnovation *");
  Serial.println ("*                                       *");
  Serial.println ("* (g)et level                           *");
  Serial.println ("* (s)et (l)evel|(d)elay value           *");
  Serial.println ("* (t)race toggle                        *");
  Serial.println ("* (r)eset board                         *");
  Serial.println ("*                                       *");
  //Serial.println ("* Set thresholds for level alarms       *");
  //Serial.println ("*                                       *");
  //Serial.println ("* [s level value]                       *");
  //Serial.println ("*                                       *");
  //Serial.println ("* Set level [LlhH] / value = 0..1023    *");
  //Serial.println ("* - L = LOLO                            *");
  //Serial.println ("* - l = LO                              *");
  //Serial.println ("* - h = HI                              *");
  //Serial.println ("* - H = HIHI                            *");
  //Serial.println ("*                                       *");
  //Serial.println ("* [g level]                             *");
  //Serial.println ("*                                       *");
  //Serial.println ("* Get level [LlhHa] as a bove, a = all  *");
  //Serial.println ("*                                       *");
  Serial.println ("*****************************************");
  Serial.println("");
}

void (*resetFunc)(void) = 0;

void parseSerial () {
  // Parse and handle communication on the serial port.
  // This function is at the end of the loop.
  char c;
  while (Serial.available()) {
    c = Serial.read();
    if (c == 0xD) {
      //Carriage return. Do something with the input
      if (nSerialBufPos >= 3) {
        if (serialBuf[0] == 's' && serialBuf[1] == ' ') {
          //Set Command
          if (nSerialBufPos < 5 || serialBuf[3] != ' ') {
            Serial.println ("Incorrect format for set command");
          } else {
            int16_t nTmp = 0;
            int32_t dTmp = 0;
            switch (serialBuf[2]) {
              case 'L':
                nTmp = atoi(&serialBuf[4]);
                if (nTmp < 0 || nTmp > 1023) {
                  Serial.print ("Illegal value: ");
                  Serial.println (nTmp);
                } else {
                  tLoLo = nTmp;
                  writeInt16 (0, tLoLo);
                  Serial.print (" LOLO set to: ");
                  Serial.println (tLoLo);
                }
                break;
              case 'l':
                nTmp = atoi(&serialBuf[4]);
                if (nTmp < 0 || nTmp > 1023) {
                  Serial.print ("Illegal value: ");
                  Serial.println (nTmp);
                } else {
                  tLo = nTmp;
                  writeInt16 (2, tLo);
                  Serial.print (" LO set to: ");
                  Serial.println (tLo);
                }
                break;
              case 'h':
                nTmp = atoi(&serialBuf[4]);
                if (nTmp < 0 || nTmp > 1023) {
                  Serial.print ("Illegal value: ");
                  Serial.println (nTmp);
                } else {
                  tHi = nTmp;
                  writeInt16 (4, tHi);
                  Serial.print (" HI set to: ");
                  Serial.println (tHi);
                }
                break;
              case 'H':
                nTmp = atoi(&serialBuf[4]);
                if (nTmp < 0 || nTmp > 1023) {
                  Serial.print ("Illegal value: ");
                  Serial.println (nTmp);
                } else {
                  tHiHi = nTmp;
                  writeInt16 (6, tHiHi);
                  Serial.print (" HIHI set to: ");
                  Serial.println (tHiHi);
                }
                break;
              case 'd':
                dTmp = atoi(&serialBuf[4]);
                if (dTmp < 0 || nTmp > 1000000) {
                  Serial.print ("Illegal value: ");
                  Serial.println (dTmp);
                } else {
                  delay_time = dTmp;
                  writeInt32 (8, delay_time);
                  Serial.print ("Delay time set to : ");
                  Serial.println (delay_time);
                }
                break;

            }
          }
        } else if (serialBuf[0] == 'g' && serialBuf[1] == ' ') {
          //Get Command
          switch (serialBuf[2]) {
            case 'L':
              Serial.println ("");
              Serial.print (">>  LOLO set to: ");
              Serial.println (tLoLo);
              Serial.println ("");
              break;
            case 'l':
              Serial.println ("");
              Serial.print (">>  LO set to: ");
              Serial.println (tLo);
              Serial.println ("");
              break;
            case 'h':
              Serial.println ("");
              Serial.print (">>  HI set to: ");
              Serial.println (tHi);
              Serial.println ("");
              break;
            case 'H':
              Serial.println ("");
              Serial.print (">>  HIHI set to: ");
              Serial.println (tHiHi);
              Serial.println ("");
              break;
            default:
              Serial.println ("");
              Serial.print (">>  LOLO set to: ");
              Serial.println (tLoLo);
              Serial.print (">>  LO set to: ");
              Serial.println (tLo);
              Serial.println ("");
              Serial.print (">>  HI set to: ");
              Serial.println (tHi);
              Serial.print (">>  HIHI set to: ");
              Serial.println (tHiHi);
              Serial.print (">>  delay set to: ");
              Serial.println (delay_time);
              break;
          }
        } else if (serialBuf[0] == 't') {
          trace = 1 - trace;
        } else if (serialBuf[0] == 'r') {
          resetFunc();
        } else if (serialBuf[0] == 'u') {
          uptime();
        } else {
          //Unknown input
          Serial.print ("Unknown command: ");
          Serial.println (serialBuf[0]);
          serialHelp();
        }
      } else {
        Serial.println ("Unknown command (too short)");
        serialHelp();
      }
      // Empty the buffer
      nSerialBufPos = 0;
      serialBuf [0] = '\0';
    } else if (c == 0xA) {
      //Line feed. Discard thisat
    } else {
      //Put it in the buffer (unless it's full)
      if (nSerialBufPos < serialBufSize) {
        serialBuf [nSerialBufPos] = c;
        nSerialBufPos++;
        serialBuf[nSerialBufPos] = '\0';
      }
    }
  }
}

// Minimal LED interface
//
// Assume 5 red leds making a 5 level meter somewhere on D2-D12
// Extension : make this configurable via Serial/EEPROM
//
// Wiring : LED, 330 ohm resistor to ground

int led_interface_enabled = 1;

int ledLoLoPin = 2;
int ledLoPin = 3;
int ledNormalPin = 4;
int ledHiPin = 5;
int ledHiHiPin = 6;

void led_interface()
{
  digitalWrite(ledLoLoPin, HIGH);

  // TODO : Make more elegant with arrays.

  if (nWarnLevel >= LEVEL_LO) {
    digitalWrite(ledLoPin, HIGH);
  } else {
    digitalWrite(ledLoPin, LOW);
  }

  if (nWarnLevel >= LEVEL_NORMAL) {
    digitalWrite(ledNormalPin, HIGH);
  } else {
    digitalWrite(ledNormalPin, LOW);
  }

  if (nWarnLevel >= LEVEL_HI) {
    digitalWrite(ledHiPin, HIGH);
  } else {
    digitalWrite(ledHiPin, LOW);
  }

  if (nWarnLevel >= LEVEL_HIHI) {
    digitalWrite(ledHiHiPin, HIGH);
  } else {
    digitalWrite(ledHiHiPin, LOW);
  }
}

// LCD Interface
//
// See e.g. https://www.arduino.cc/en/Tutorial/HelloWorld
//
// Physical interface requires : 6 digital outputs
//
// RS reset pin

int lcd_interface_enabled;

int lcdResetPin = 7;
int lcdEnablePin = 8;
int lcdD4Pin = 9;
int lcdD5Pin = 10;
int lcdD6Pin = 11;
int lcdD7Pin = 12;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(lcdResetPin, lcdEnablePin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

void lcd_interface()
{
  sprintf (LcdTopRowBuffer, "%s: level A0: %4i", msgPrefix[nWarnLevel], nCurrentLevel);
  sprintf (LcdBottomRowBuffer, "%s: level A0: %4i", msgPrefix[nWarnLevel], nCurrentLevel);
}

int32_t t_start;
volatile int32_t t_last;
int32_t cycle_t;

void setup() {
  t_start = millis();
  t_last = millis();
  pinMode (A0, INPUT);
  pinMode (LED_BUILTIN, OUTPUT);
  // start the Ethernet and UDP:
  Ethernet.begin(mac, ip, gw, nm);
  Udp.begin(localPort);

  Serial.begin(115200);

  serialHelp();

  //Get trigger levels from EEPROM
  tLoLo = readInt16 (0);
  tLo = readInt16 (2);
  tHi = readInt16 (4);
  tHiHi = readInt16(6);

  Serial.print ("tLoLo = ");
  Serial.println (tLoLo);
  Serial.print ("tLo = ");
  Serial.println (tLo);
  Serial.print ("tHi = ");
  Serial.println (tHi);
  Serial.print ("tHiHi = ");
  Serial.println (tHiHi);

  delay_time = readInt32(8);

  Serial.print ("delay (EEPROM, 8) = ");
  Serial.println (delay_time);

  simWriteInt32(8, 1001);
  simReadInt32(8);
  simReadInt32(0);

  if (delay_time < 1000) {
    delay_time = 1000;
  }

  Serial.print ("delay = ");
  Serial.println (delay_time);

  serialBuf[0] = '\0';


  if (led_interface_enabled) {
    pinMode(ledLoLoPin, OUTPUT);
    pinMode(ledLoPin, OUTPUT);
    pinMode(ledNormalPin, OUTPUT);
    pinMode(ledHiPin, OUTPUT);
    pinMode(ledHiHiPin, OUTPUT);

    // Extension : organise the pins in an array
    // Extension : flash the LED pins at startup or run up/down a couple of times.
  }

  if (lcd_interface_enabled) {
    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.print("Water Level");
    lcd.setCursor(0, 1);
    sprintf (LcdBottomRowBuffer, "%-7s A0: %4i", msgPrefix[nWarnLevel], nCurrentLevel);
    lcd.print(LcdBottomRowBuffer);
  }

}

int udp_interface_enabled = 1;
int loop_iterations = 0;

void loop() {
  loop_iterations++;
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);

  nCurrentLevel = analogRead (A0);

  if (nCurrentLevel < tLoLo) {
    nWarnLevel = 0;
  } else if (nCurrentLevel < tLo) {
    nWarnLevel = 1;
  } else if (nCurrentLevel < tHi) {
    nWarnLevel = 2;
  } else if (nCurrentLevel < tHiHi) {
    nWarnLevel = 3;
  } else if (nCurrentLevel >= tHiHi) {
    nWarnLevel = 4;
  }

  if (trace) {
    Serial.print("Loop end : ");
    Serial.println(loop_iterations);
    Serial.print("Iteration time: ");
    cycle_t = millis() - t_last;
    Serial.println(cycle_t);
    t_last = millis();
    Serial.print("Time since reset : ");
    Serial.println(t_last);
  }


  if (led_interface_enabled) {
    led_interface();
  }

  if (lcd_interface_enabled) {
    lcd_interface();
  }


  if (udp_interface_enabled) {

    sprintf (ReplyBuffer, "%s: level A0: %4i", msgPrefix[nWarnLevel], nCurrentLevel);
    // Udp.sendPacket( ReplyBuffer, remoteIp, remotePort);
    //Udp.beginPacket(remoteIp, remotePort);
    //Udp.write(ReplyBuffer);
    //Udp.endPacket();
    Serial.println(ReplyBuffer);

  }


  if (Serial.available()) {
    parseSerial();
  }

  delay (delay_time);
}

