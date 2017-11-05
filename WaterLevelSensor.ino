
/**
  WaterLevelSensor.ino, Adam Stephen, https://github.com/AdamVStephen/arduino

  git clone git@github.com:AdamVStephen/arduino
*/

// TODO: avoid memory writes if unnecessary.
// Add a Serial 'z' : to zero the statistics.

// Key constants
// Stackoverflow discussion recommends const int or enum, and not #define
// To investigate : compile time elimination of unused code.

enum Config {
  DEBUG = 0,
  BAUDRATE = 115200,
  SCROLL_DELAY = 250,
  LEVEL_CM_MAX_THRESHOLD = 50,
  LEVEL_CM_MIN_THRESHOLD = 0,
  MAX_DELAY_MS = 1000000
};

#define LED_IF_ENABLED 1
#define LCD_IF_ENABLED 1
#define UDP_IF_ENABLED 0

const int led_interface_enabled = LED_IF_ENABLED;
const int lcd_interface_enabled = LCD_IF_ENABLED;
const int udp_interface_enabled = UDP_IF_ENABLED;

int loops;

#if UDP_IF_ENABLED
warblesnook();
#endif

/** Documentation

  N.B. : Arduino has issues with sprintf(buf, "%f", float) : use String(float).c_str() or equivalent.

  TODO list :

  - Make low power
  - Add a configuration parameter for the delay time
  - Make the configuration interface generic
  - Fix the UDP issue (random delays/timeouts)
  - Calibration from DAC to mm
  - Enforce LOLO < LO < HI < HIHI

  Provenance:

  1. Initially derived from WaterLevelSensorUdp by Fredrik Moberg, http://brelovich.com/  (which in turn credits 2.)
  2. <which itself was> Based on Udp sample code created 21 Aug 2010 by Michael Margolis

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
  (g) LCD 16x2 display        [Implemented : Y]

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

  Physical Design Info :

  Bill of materials plus key dimensions :

    - Arduino Nano board : []
    - LCD 1602 panel : []
    - 10k potentiometer to control LCD brightness/contrast
    - MPX5010DP differential pressure sensor
    - Smoothing capacitors 1uF, 10pF, 470pF
    - 5V to 3.3V regulator

  Key pinout information

  - MPX5010DP - with the manufacturer printing marking top surface
  -- Top pressure port (RHS) = measured pressure (0-10kPa)
  -- Bottom pressure port (LHS) = reference pressure
  -- 6 pins from left to right are : V_OUT, GND, VCC (5V), V1, V2, V_EX
  -- Datasheet from NXP corp does not document V1, V2, V_EX but V_OUT is linearly proportional to pressure
  -- 0.2-4.7V output at max 1kHz, accuracy is 5%

  Calibration information  (based on data sheet)

  -- Transfer function V_OUT = V_S * ( 0.09 P + 0.04 ) or P = ((V_OUT/V_S) - 0.04))/0.09 kPa.   i.e. 4.7 V = 10 kPa, 0.2V = 0 kPa
  -- V_S is required to be 5.0 V (to check - performance if supply voltage drops)
  -- In SI units : P (Pa) = 2222.22 * (V_OUT - 0.2)
  -- Assuming tank open to atmosphere, then we expect hydrostatic pressure P = rho g h = 9,81 * 1000 * h(m).
  -- Full tank at 50cm should read approximately 4905 Pa and V_OUT = 2.40725
  -- Calibration formulae
  -- A0 is a 12 bit ADC 0-1023.
  -- Tank head in m = Pressure / 9810 ~= (0.1085 V_adc - 0.4444) / 9.81   where V_adc is the ADC integer readback of the sensor voltage
  -- Recommended supply circuit should have two capacitors parallel between VCC/GND of values 1uF and 0.01uF (10pF)
  -- Smoothing capacitor of 470pF across V_OUT/GND
  -- Accuracy is +- 5%

  LCD Wiring

   LCD RS pin to digital pin lcsResetPin (e.g. 7)
   LCD Enable pin to digital pin lcdEnablePin (e.g. 8)
   LCD D4 pin to digital pin lcdD4Pin (e.g. 9)
   LCD D5 pin to digital pin lcdD5Pin (e.g. 10)
   LCD D6 pin to digital pin lcdD6Pin (e.g. 11)
   LCD D7 pin to digital pin lcdD7Pin (e.g. 12)
   LCD R/W pin to ground
   LCD VSS pin to ground
   LCD VCC pin to 5V
   10K resistor pot
   ends to +5V and ground
   wiper to LCD VO pin (pin 3 on LCD)


*/

#include <Ethernet.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <EthernetClient.h>
#include <Dhcp.h>
#include <Dns.h>
#include <SPI.h>         // needed for Arduino versions later than 0018
#include <Ethernet.h>
#include <Udp.h>         // UDP library from: bjoern@cs.stanford.edu 12/30/2008
#include <stdio.h>
#include <EEPROM.h>           //We will read and write config from here
#include <LiquidCrystal.h>

int trace = 0;

#if UDP_IF_ENABLED
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

#endif

// Trigger levels for alarms : by inference tNormal is tLo < level < tHi
// Will be fetched from eeprom
// Reference to height of tank in cm
int tLoLo = 5;
int tLo = 10;
int tHi = 40;
int tHiHi = 45;

int32_t delay_time = 0;

//Message prefixes
char msgPrefix[5][7] = { "LOLO ", "LO ", "NORMAL", "HI", "HIHI" };
byte nWarnLevel = 0;

enum Warnlevel {
  LEVEL_LOLO = 0,
  LEVEL_LO = 1,
  LEVEL_NORMAL = 2,
  LEVEL_HI = 3,
  LEVEL_HIHI = 4
};

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

// Write 4 bytes to EEPROM, starting at the given address.

void writeInt32 (uint32_t nAddress, int32_t nData) {

  EEPROM.write (nAddress + 0, ((nData & 0xFF000000) >> 24));
  EEPROM.write (nAddress + 1, ((nData & 0x00FF0000) >> 16));
  EEPROM.write (nAddress + 2, ((nData & 0x0000FF00) >> 8));
  EEPROM.write (nAddress + 3, ((nData & 0x000000FF) >> 0));

  //
#if DEBUG
  Serial.println("writeInt32 +0 +1 +2 +3");
  Serial.println(nAddress);
  Serial.println(nData);
  Serial.println((nData & 0xFF000000) >> 24);
  Serial.println((nData & 0x00FF0000) >> 16);
  Serial.println((nData & 0x0000FF00) >> 8);
  Serial.println(nData & 0x000000FF);
#endif
  //
  return;
}

// Read 4 bytes from EEPROM, starting at the given address.

int32_t readInt32 (uint32_t nAddress) {

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

#if DEBUG
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
#endif


/**
   Sensor Transformations

   Range 0.2V = 0 kPa to 4.7V = 10.0 kPa
   A0 is 12 bit 0-1023

   P (Pa) = rho * g * h
*/

const float VCC = 5.0;
const float MAX_KPA = 10.0;
const float COEFF_LIN_KPA = 0.09;
const float COEFF_OFFSET_KPA = 0.04;
// Min Vout 0.2 for standard values above
const float MIN_VOUT = (VCC * COEFF_OFFSET_KPA);
// Max Vout 4.7 for standard values above
const float MAX_VOUT = (VCC * ((COEFF_LIN_KPA * MAX_KPA) + COEFF_OFFSET_KPA));

// Physical constants
// Gravitational acceleration ms^-2
const float G_ACC = 9.81;
// Density of water
const float RHO_WATER = 1000.0;

float digitalToVout(long d) {
  return (VCC * d) / 1023.0;
}

float voutToDigital(float v) {
  return (v * 1023.0) / VCC;
}

float kpaToVout(float kpa) {
  // Clip to max range
  if (kpa > MAX_KPA) {
    kpa = MAX_KPA;
  }
  return VCC * (COEFF_LIN_KPA * kpa + COEFF_OFFSET_KPA);
}

float voutToKpa(float v) {
  // Clip to range
  if (v < MIN_VOUT) {
    v = MIN_VOUT;
  }
  if (v > MAX_VOUT) {
    v = MAX_VOUT;
  }
  return ( v - MIN_VOUT ) / (VCC * COEFF_LIN_KPA);
}

float voutToPa(float v) {
  return 1000.0 * voutToKpa(v);
}

float kpaToLevelMeters(float kpa) {
  return (kpa * 1000.) / (RHO_WATER * G_ACC);
}

float kpaToLevelCentimeters(float kpa) {
  return (100.0 * kpaToLevelMeters(kpa));
}

int digitalToLevelCentimeters(long d) {
  return int(kpaToLevelCentimeters(voutToKpa(digitalToVout(d))));
}

char serbuf[50];

void serialHelp() {
  Serial.println("");
  Serial.println ("*****************************************");
  Serial.println ("* Serial Interface to WaterLevelMonitor *");
  Serial.println ("*                                       *");
  Serial.println ("* Adam Stephen, Infinnovation Ltd.      *");
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
            // TODO
            switch (serialBuf[2]) {
              case 'L':
                nTmp = atoi(&serialBuf[4]);
                tLoLo = constrain(nTmp, LEVEL_CM_MIN_THRESHOLD, LEVEL_CM_MAX_THRESHOLD);
                // what does atoi return in the case of illegal input ?
                writeInt16 (0, tLoLo);
                Serial.print (" LOLO set to: ");
                Serial.println (tLoLo);
                break;
              case 'l':
                nTmp = atoi(&serialBuf[4]);
                tLo = constrain(nTmp, LEVEL_CM_MIN_THRESHOLD, LEVEL_CM_MAX_THRESHOLD);
                writeInt16 (2, tLo);
                Serial.print (" LO set to: ");
                Serial.println (tLo);
                break;
              case 'h':
                nTmp = atoi(&serialBuf[4]);
                tHi = constrain(nTmp, LEVEL_CM_MIN_THRESHOLD, LEVEL_CM_MAX_THRESHOLD);
                writeInt16 (4, tHi);
                Serial.print (" HI set to: ");
                Serial.println (tHi);
                break;
              case 'H':
                nTmp = atoi(&serialBuf[4]);
                tHiHi = constrain(nTmp, LEVEL_CM_MIN_THRESHOLD, LEVEL_CM_MAX_THRESHOLD);
                writeInt16 (6, tHiHi);
                Serial.print (" HIHI set to: ");
                Serial.println (tHiHi);
                break;
              case 'd':
                dTmp = atoi(&serialBuf[4]);
                delay_time = constrain(dTmp, 0, MAX_DELAY_MS);
                writeInt32 (8, delay_time);
                Serial.print ("Delay time set to : ");
                Serial.println (delay_time);
                break;
            }
          }
        } else if (serialBuf[0] == 'g' && serialBuf[1] == ' ') {
          //Get Command
          switch (serialBuf[2]) {
            case 'L':
              sprintf(serbuf, ">> LOLO set to : %d", tLoLo); Serial.println (serbuf);
              break;
            case 'l':
              sprintf(serbuf, ">> LO set to : %d", tLo); Serial.println (serbuf);
              break;
            case 'h':
              sprintf(serbuf, ">> HI set to : %d", tHi); Serial.println (serbuf);
              break;
            case 'H':
              sprintf(serbuf, ">> HIHI set to : %d", tHiHi); Serial.println (serbuf);
              break;
            default:
              sprintf(serbuf, ">> LOLO set to : %d", tLoLo); Serial.println (serbuf);
              sprintf(serbuf, ">> LO set to : %d", tLo); Serial.println (serbuf);
              sprintf(serbuf, ">> HI set to : %d", tHi); Serial.println (serbuf);
              sprintf(serbuf, ">> HIHI set to : %d", tHiHi); Serial.println (serbuf);
              sprintf(serbuf, ">> delay_time set to : %d", delay_time); Serial.println (serbuf);
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

#if LED_IF_ENABLED
const int ledLoLoPin = 2;
const int ledLoPin = 3;
const int ledNormalPin = 4;
const int ledHiPin = 5;
const int ledHiHiPin = 6;

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
#endif

// LCD Interface
//
#if LCD_IF_ENABLED

int lcdResetPin = 7;
int lcdEnablePin = 8;
int lcdD4Pin = 9;
int lcdD5Pin = 10;
int lcdD6Pin = 11;
int lcdD7Pin = 12;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(lcdResetPin, lcdEnablePin, lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

// Define 8 custom 5x8 characters for the LCD consisting of filled 5 columns and increasing height
// Used to give a level meter within one character.
// See calls to lcd.createChar(int, byte) in setup()

byte level_01[8] = {0, 0, 0, 0, 0, 0, 0, 31};
byte level_02[8] = {0, 0, 0, 0, 0, 0, 31, 31};
byte level_03[8] = {0, 0, 0, 0, 0, 31, 31, 31};
byte level_04[8] = {0, 0, 0, 0, 31, 31, 31, 31};
byte level_05[8] = {0, 0, 0, 31, 31, 31, 31, 31};
byte level_06[8] = {0, 0, 31, 31, 31, 31, 31, 31};
byte level_07[8] = {0, 31, 31, 31, 31, 31, 31, 31};
byte level_08[8] = {31, 31, 31, 31, 31, 31, 31, 31};


/**
     Display arithmetic
*/

char topTitle[] = "H20 (cm) :";
int topTitleLen = String(topTitle).length();

int pressureBar(int d) {

#if DEBUG_TRACE
  char sbuf[50];
  sprintf(sbuf, "d %d v %f range %f - %f map 0,8", d, String(digitalToVout(d)).c_str(), String(MIN_VOUT).c_str(), String(MAX_VOUT).c_str());
  Serial.println(sbuf);
  Serial.print(digitalToVout(d));
  Serial.println();
  Serial.print(MIN_VOUT);
  Serial.println();
  Serial.print(MAX_VOUT);
  Serial.println();
#endif

  return constrain(map(digitalToVout(d), MIN_VOUT, MAX_VOUT, 0, 7), 0, 7);
}

int heightBar(int h) {
  return constrain(map(h, 0, 50, 0, 7), 0, 7);
}

char lcdbuf0[16];
char lcdbuf1[16];


void lcdHelp() {
  char buf[16] = "0123456789ABCEF";
  char * messages[] = {
    "Options for help"
    "Press s to set  ",
    "Press r to reset",
    "Press c to calib",
    "                "
  };
  for (int i = 0; i < 4; i++) {
    lcd.setCursor(0, 1);
    lcd.print(messages[i]);
    delay(2000);
  }
  lcd.setCursor(0, 1);
  lcd.print("                ");
}

int hlo = 100;
int hhi = 0;
const int numReadings = 10;
int readings[numReadings];
int readIndex = 0;
int total = 0;
int hav = 0;

/**
   Setup -----------------------------------------------------------------------------------
*/
char dbugbuf[50];



void lcd_interface(int v_adc, int loops) {

  int h = digitalToLevelCentimeters(v_adc);
  lcd.setCursor(0, 0);
  sprintf(lcdbuf0, "%s%02d", topTitle, h);
  lcd.print(lcdbuf0);
  //lcd.print(h);
  int hbar = heightBar(h);
  lcd.setCursor(topTitleLen + 2 + 1, 0);
  lcd.write((byte)hbar);

  int j = loops % 8;
  lcd.setCursor(15, 0);
  lcd.write((byte)(7 - j));
  lcd.setCursor(15, 1);
  lcd.write((byte)(7 - j));

  // Update statistics
  if (h < hlo) {
    hlo = h;
  }
  if (h > hhi) {
    hhi = h;
  }
  total = total - readings[readIndex];
  readings[readIndex] = h;
  total = total + readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  float av = total / numReadings;
  hav = (int) av;
  sprintf(lcdbuf1, "%02d-%02d Avg %02d", hlo, hhi, hav);
  lcd.setCursor(0, 1);
  lcd.print(lcdbuf1);
  int avbar = heightBar(hav);
  lcd.setCursor(13, 1);
  lcd.write((byte)avbar);

  sprintf(serbuf, "Analogue value %d Bar %d", v_adc, avbar);
  Serial.println(serbuf);

  int tms = millis();
  int uptimeSS = tms / 1000;
  int uptimeMM = uptimeSS % 60;
  int uptimeHH = uptimeSS / 3600;

  if (loops % 9 == 0) {
    if (trace) {
      lcdHelp();
    }
  }

  if (loops % 10 == 0 ) {
    if (nWarnLevel != LEVEL_NORMAL) {
      sprintf (lcdbuf1, "** %s: %02d **", msgPrefix[nWarnLevel], h);
      for (int w = 0; w < 3; w++) {
        lcd.setCursor(0, 1);
        lcd.print(lcdbuf1);
        delay(1000);
        lcd.setCursor(0, 1);
        lcd.print("                ");
        delay(1000);
      }
    }
  } else if (loops % 37 == 0) {
    lcd.setCursor(0, 1);
    sprintf(lcdbuf1, "%04d %02d:%02d     ", loops, uptimeHH, uptimeSS);
    lcd.print(lcdbuf1);
    delay(3000);
  } else if (loops %  1729 == 0) {
    lcd.setCursor(0, 1);
    lcd.print("1729 : Ramanujan");
    delay(2000);
    lcd.setCursor(0, 1);
    lcd.print("=1^3 + 12^3     ");
    delay(2000);
    lcd.setCursor(0, 1);
    lcd.print("=9^3 + 10^3     ");
    delay(2000);
  } else if (loops % 4695 == 0) {
    lcd.setCursor(0, 1);
    lcd.print("HappyBoatingNick");
    delay(2000);
  }
}


#endif // LCD_IF_ENABLED

int32_t t_start;
volatile int32_t t_last;
int32_t cycle_t;

void setup() {
  Serial.begin(BAUDRATE);
  t_start = millis();
  t_last = millis();
  pinMode (A0, INPUT);
  pinMode (LED_BUILTIN, OUTPUT);

#if LCD_IF_ENABLED
  if (lcd_interface_enabled) {
    lcd.createChar(0, level_01);
    lcd.createChar(1, level_02);
    lcd.createChar(2, level_03);
    lcd.createChar(3, level_04);
    lcd.createChar(4, level_05);
    lcd.createChar(5, level_06);
    lcd.createChar(6, level_07);
    lcd.createChar(7, level_08);

    lcd.begin(16, 2);
    lcd.setCursor(0, 0);
    lcd.print("GEM Water Meter");
    lcd.setCursor(0, 1);
    lcd.print("Initialising HW");
    delay(1000);
    for (int i = 0; i <= 16; i++) {
      lcd.scrollDisplayLeft();
      delay(SCROLL_DELAY);
    }
    for (int i = 0; i <= 16; i++) {
      lcd.scrollDisplayRight();
      delay(SCROLL_DELAY);
    }
    delay(1000);
    lcd.clear();
  } // lcd_interface_enabled
#endif

  // start the Ethernet and UDP:
#if UDP_IF_ENABLED1
  Ethernet.begin(mac, ip, gw, nm);
  Udp.begin(localPort);
#endif

  serialHelp();

  //Get trigger levels from EEPROM

  tLoLo = readInt16 (0);
  tLo = readInt16 (2);
  tHi = readInt16 (4);
  tHiHi = readInt16(6);
  delay_time = readInt32(8);
#if DEBUG
  Serial.print ("tLoLo = ");
  Serial.println (tLoLo);
  Serial.print ("tLo = ");
  Serial.println (tLo);
  Serial.print ("tHi = ");
  Serial.println (tHi);
  Serial.print ("tHiHi = ");
  Serial.println (tHiHi);
  Serial.print ("delay (EEPROM, 8) = ");
  Serial.println (delay_time);
#endif

  if (delay_time < 1000) {
    delay_time = 1000;
  }
#if DEBUG
  Serial.print ("delay = ");
  Serial.println (delay_time);
#endif

  serialBuf[0] = '\0';

#if LED_IF_ENABLED
  if (led_interface_enabled) {
    pinMode(ledLoLoPin, OUTPUT);
    pinMode(ledLoPin, OUTPUT);
    pinMode(ledNormalPin, OUTPUT);
    pinMode(ledHiPin, OUTPUT);
    pinMode(ledHiHiPin, OUTPUT);

    // Extension : organise the pins in an array
    // Extension : flash the LED pins at startup or run up/down a couple of times.
  } // led_interface_enabled
#endif

  loops = 4695 - 30;

} // setup

int v_adc;
int h_cm;

void loop() {

  loops++;

  v_adc = analogRead (A0);
  h_cm = digitalToLevelCentimeters(v_adc);

  if (h_cm < tLoLo) {
    nWarnLevel = LEVEL_LOLO;
  } else if (h_cm < tLo) {
    nWarnLevel = LEVEL_LO;
  } else if (h_cm < tHi) {
    nWarnLevel = LEVEL_NORMAL;
  } else if (h_cm < tHiHi) {
    nWarnLevel = LEVEL_HI;
  } else if (h_cm >= tHiHi) {
    nWarnLevel = LEVEL_HIHI;
  }

  if (trace) {
    Serial.print("Loop end : ");
    Serial.println(loops);
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
    lcd_interface(v_adc, loops);
  }

#if UDP_IF_ENABLED
  if (udp_interface_enabled) {

    sprintf (ReplyBuffer, "%s: level A0: %4i", msgPrefix[nWarnLevel], v_adc);
    // Udp.sendPacket( ReplyBuffer, remoteIp, remotePort);
    //Udp.beginPacket(remoteIp, remotePort);
    //Udp.write(ReplyBuffer);
    //Udp.endPacket();
    Serial.println(ReplyBuffer);
  }
#endif

  if (Serial.available()) {
    parseSerial();
  }

  delay (delay_time);
}

