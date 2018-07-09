/**
  OilLevelSensor.ino, Adam Stephen, https://github.com/AdamVStephen/gem-water-level-gauge
  *
  * TODO: Migrate the repo to more general : "gem-level-gauge".
  * TODO: Document tank limits for the pressure sensor as a function of density.
*/

char release[] = "1.6"; // TODO : git commit/release process to populate this field.

// V1.0 Derived from WaterLevelSensor 1.5.6, stripped down, reconfigured for LCD Shield pinout and JET oil level density.

// JET Oil version simulation runs the max threshold below to 125.
// Cleanup : make consistent with the option to toggle Water/Oil (POC/Gimmick)

enum Config {
  DEBUG = 0,
  BAUDRATE = 115200,
  SCROLL_DELAY = 250,
  LEVEL_CM_MAX_THRESHOLD = 125,
  LEVEL_CM_MIN_THRESHOLD = 0,
  MAX_DELAY_MS = 1000000
};

#define LED_IF_ENABLED 0
#define LCD_IF_ENABLED 1
#define UDP_IF_ENABLED 0

const int led_interface_enabled = LED_IF_ENABLED;
const int lcd_interface_enabled = LCD_IF_ENABLED;
const int udp_interface_enabled = UDP_IF_ENABLED;

unsigned long loops;

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

#if UDP_IF_ENABLED
#include <Ethernet.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <EthernetClient.h>
#include <Dhcp.h>
#include <Dns.h>
#endif

#include <SPI.h>         // needed for Arduino versions later than 0018

#include <stdio.h>
#include <EEPROM.h>           //We will read and write config from here
#include <LiquidCrystal.h>

int trace = 0;
int simulation = 0;

// Trigger levels for alarms : by inference tNormal is tLo < level < tHi
// Will be fetched from eeprom
// Reference to height of tank in cm
int tLoLo = 5;
int tLo = 10;
int tHi = 90;
int tHiHi = 110;

int32_t delay_time = 0;

//Message prefixes
char msgPrefix[5][7] = { "LOLO ", "LO ", "NORMAL", "HI", "HIHI" };

enum WarnLevel {
  LEVEL_LOLO = 0,
  LEVEL_LO = 1,
  LEVEL_NORMAL = 2,
  LEVEL_HI = 3,
  LEVEL_HIHI = 4
};

WarnLevel nWarnLevel = LEVEL_NORMAL;

/*
    TODO /Read/Write to EEProm mixed with non standard int type... cleanup

  enum AlarmStatus {
  ALARM_DISABLED = 0,
  ALARM_ASSERT = 1,
  ALARM_FLASH = 2
  };

  AlarmStatus loloEnabled = ALARM_FLASH;
  AlarmStatus loEnabled = ALARM_FLASH;
  AlarmStatus hiEnabled = ALARM_FLASH;
  AlarmStatus hihiEnabled = ALARM_FLASH;
*/
#define ALARM_DISABLED 0
#define ALARM_ASSERT 1
#define ALARM_FLASH 2

int loloEnabled = ALARM_FLASH;
int loEnabled = ALARM_FLASH;
int hiEnabled = ALARM_FLASH;
int hihiEnabled = ALARM_FLASH;

//Character buffer and pointer(s) for Serial command parsing
const int serialBufSize = 20;
char serialBuf [serialBufSize + 1];
unsigned int nSerialBufPos = 0;

// Caveat : EEPROM memory has a specified life of 100k write/erase cycles
//
// SO : only support changing parameters infrequently !
//
// Takes 3.3ms per write()

#define EEPROM_OFFSET_TLOLO 0
#define EEPROM_OFFSET_TLO 2
#define EEPROM_OFFSET_THI 4
#define EEPROM_OFFSET_THIHI 6
#define EEPROM_OFFSET_DELAY 8
#define EEPROM_OFFSET_ELOLO 12
#define EEPROM_OFFSET_ELO 14
#define EEPROM_OFFSET_EHI 16
#define EEPROM_OFFSET_EHIHI 18

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
  Serial.print("Release :");
  Serial.println(release);
}


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
const float RHO_JET_OIL = 870.0;

const char * s_assert = "assert";
const char * s_disabled = "disable";
const char * s_flash = "flash";

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

float kpaToWaterLevelMeters(float kpa) {
  return (kpa * 1000.) / (RHO_WATER * G_ACC);
}

float kpaToJetOilLevelMeters(float kpa) {
  return (kpa * 1000.) / (RHO_JET_OIL * G_ACC);
}


#define LIQUID_WATER 0
#define LIQUID_JET_OIL 1

int liquidMode = LIQUID_JET_OIL;

float kpaToLevelCentimeters(float kpa) {
  if (liquidMode == LIQUID_WATER) {
    return (100.0 * kpaToWaterLevelMeters(kpa));
  } else if (liquidMode == LIQUID_JET_OIL) {
    return (100.0 * kpaToJetOilLevelMeters(kpa));
  }
}

int digitalToLevelCentimeters(long d) {
  return int(kpaToLevelCentimeters(voutToKpa(digitalToVout(d))));
}

char serbuf[50];

void serialHelp() {
  Serial.println("");
  //Serial.println ("*****************************************");
  //Serial.println ("* Serial Interface to WaterLevelMonitor *");
  //Serial.println ("*                                       *");
  Serial.println (F("* Adam Stephen, Infinnovation Ltd.      *"));
  Serial.println (F("*                                       *"));
  Serial.println (F("* (g)et level                           *"));
  Serial.println (F("* (s)et (l)evel|(d)elay value           *"));
  Serial.println (F("* (t)race toggle                        *"));
  Serial.println (F("* (r)eset board                         *"));
  Serial.println (F("*                                       *"));
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
  //Serial.println ("*****************************************");
  Serial.println("");
}

void (*resetFunc)(void) = 0;

void parseSerial () {
  // Parse and handle communication on the serial port.
  // This function is at the end of the loop.
  char c;

  while (Serial.available()) {
    c = Serial.read();
    //sprintf(serbuf, "Serial buffer contains[%s]", serialBuf);
    //Serial.println(serbuf);
    if (c == 0xD) {
      //Carriage return. Do something with the input
      // Buffer has 3 elements : cmd space arg
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
                writeInt16 (EEPROM_OFFSET_TLOLO, tLoLo);
                Serial.print (" LOLO set to: ");
                Serial.println (tLoLo);
                break;
              case 'l':
                nTmp = atoi(&serialBuf[4]);
                tLo = constrain(nTmp, LEVEL_CM_MIN_THRESHOLD, LEVEL_CM_MAX_THRESHOLD);
                writeInt16 (EEPROM_OFFSET_TLO, tLo);
                Serial.print (" LO set to: ");
                Serial.println (tLo);
                break;
              case 'h':
                nTmp = atoi(&serialBuf[4]);
                tHi = constrain(nTmp, LEVEL_CM_MIN_THRESHOLD, LEVEL_CM_MAX_THRESHOLD);
                writeInt16 (EEPROM_OFFSET_THI, tHi);
                Serial.print (" HI set to: ");
                Serial.println (tHi);
                break;
              case 'H':
                nTmp = atoi(&serialBuf[4]);
                tHiHi = constrain(nTmp, LEVEL_CM_MIN_THRESHOLD, LEVEL_CM_MAX_THRESHOLD);
                writeInt16 (EEPROM_OFFSET_THIHI, tHiHi);
                Serial.print (" HIHI set: ");
                Serial.println (tHiHi);
                break;
              case 'd':
                dTmp = atoi(&serialBuf[4]);
                delay_time = constrain(dTmp, 0, MAX_DELAY_MS);
                writeInt32 (EEPROM_OFFSET_DELAY, delay_time);
                Serial.print ("Delay time set: ");
                Serial.println (delay_time);
                break;
            }
          }
        } else if (serialBuf[0] == 'e' && serialBuf[1] == ' ') {
          //Set Command
          if (nSerialBufPos < 5 || serialBuf[3] != ' ') {
            Serial.println ("Wrong format for enable command");
          } else {
            int16_t nTmp = 0;
            int32_t dTmp = 0;
            // TODO
            switch (serialBuf[2]) {
              case 'L':
                nTmp = atoi(&serialBuf[4]);
                loloEnabled = constrain(nTmp, ALARM_DISABLED, ALARM_FLASH);
                // what does atoi return in the case of illegal input ?
                writeInt16 (EEPROM_OFFSET_ELOLO, loloEnabled);
                Serial.print (" LOLO enable: ");
                Serial.println (loloEnabled);
                break;
              case 'l':
                nTmp = atoi(&serialBuf[4]);
                loEnabled = constrain(nTmp, ALARM_DISABLED, ALARM_FLASH);
                writeInt16 (EEPROM_OFFSET_ELO, loEnabled);
                Serial.print (" LO enable : ");
                Serial.println (loEnabled);
                break;
              case 'h':
                nTmp = atoi(&serialBuf[4]);
                hiEnabled = constrain(nTmp, ALARM_DISABLED, ALARM_FLASH) ;
                writeInt16 (EEPROM_OFFSET_EHI, hiEnabled);
                Serial.print (" HI enable: ");
                Serial.println (hiEnabled);
                break;
              case 'H':
                nTmp = atoi(&serialBuf[4]);
                hihiEnabled = constrain(nTmp, ALARM_DISABLED, ALARM_FLASH);
                writeInt16 (EEPROM_OFFSET_EHI, hihiEnabled);
                Serial.print (" HIHI enable: ");
                Serial.println (hihiEnabled);
                break;
            }
          }
        } else if (serialBuf[0] == 'g' && serialBuf[1] == ' ') {
          //Get Command
          switch (serialBuf[2]) {
            case 'L':
              sprintf(serbuf, ">> LOLO set to : %d", tLoLo); Serial.println (serbuf);
              sprintf(serbuf, ">> LOLO alarm is");
              switch (loloEnabled) {
                case ALARM_DISABLED:
                  sprintf(serbuf, s_disabled); Serial.println(serbuf);
                  break;
                case ALARM_ASSERT:
                  sprintf(serbuf, s_assert); Serial.println(serbuf);
                  break;
                case ALARM_FLASH:
                  sprintf(serbuf, s_flash); Serial.println(serbuf);
                  break;
              }
              break;
            case 'l':
              sprintf(serbuf, ">> LO set to : %d", tLo); Serial.println (serbuf);
              sprintf(serbuf, ">> LO alarm status is");
              switch (loEnabled) {
                case ALARM_DISABLED:
                  sprintf(serbuf, s_disabled); Serial.println(serbuf);
                  break;
                case ALARM_ASSERT:
                  sprintf(serbuf, s_assert); Serial.println(serbuf);
                  break;
                case ALARM_FLASH:
                  sprintf(serbuf, s_flash); Serial.println(serbuf);
                  break;
              }
              break;
            case 'h':
              sprintf(serbuf, ">> HI set to : %d", tHi); Serial.println (serbuf);
              sprintf(serbuf, ">> HI alarm status is");
              switch (hiEnabled) {
                case ALARM_DISABLED:
                  sprintf(serbuf, s_disabled); Serial.println(serbuf);
                  break;
                case ALARM_ASSERT:
                  sprintf(serbuf, s_assert); Serial.println(serbuf);
                  break;
                case ALARM_FLASH:
                  sprintf(serbuf, s_flash); Serial.println(serbuf);
                  break;
              }
              break;
            case 'H':
              sprintf(serbuf, ">> HIHI set to : %d", tHiHi); Serial.println (serbuf);
              sprintf(serbuf, ">> HIHI alarm status is");
              switch (hihiEnabled) {
                case ALARM_DISABLED:
                  sprintf(serbuf, s_disabled); Serial.println(serbuf);
                  break;
                case ALARM_ASSERT:
                  sprintf(serbuf, s_assert); Serial.println(serbuf);
                  break;
                case ALARM_FLASH:
                  sprintf(serbuf, s_flash); Serial.println(serbuf);
                  break;
              }
              break;
            default:
              sprintf(serbuf, ">> LOLO set to : %d mode %d", tLoLo, loloEnabled); Serial.println (serbuf);
              sprintf(serbuf, ">> LO set to : %d mode %d", tLo, loEnabled); Serial.println (serbuf);
              sprintf(serbuf, ">> HI set to : %d mode %d", tHi, hiEnabled); Serial.println (serbuf);
              sprintf(serbuf, ">> HIHI set to : %d mode %d", tHiHi, hihiEnabled); Serial.println (serbuf);
              sprintf(serbuf, ">> delay_time set to : %d", delay_time); Serial.println (serbuf);
              break;
          }
        }
      } else {
        // Fewer than 3 characters
        if (serialBuf[0] == 't') {
          trace = 1 - trace;
        } else if (serialBuf[0] == 'z') {
          simulation = 1 - simulation;
          Serial.println("Toggle simulation to %d"); Serial.println(simulation);
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

// LCD Interface
//
#if LCD_IF_ENABLED

int lcdResetPin = 8;
int lcdEnablePin = 9;
int lcdD4Pin = 4;
int lcdD5Pin = 5;
int lcdD6Pin = 6;
int lcdD7Pin = 7;

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

//char topTitle[] = "H20 (cm) :";
// Title Strings must all have the same length, or adapt the character handling code.
char topTitleWater[]  = "Water cm :";
char topTitleJetOil[] = "Level cm :";
char simTitle[] = "SimMode  :";
int topTitleLen = String(topTitleWater).length();

int pressureBar(int d) {

  return constrain(map(digitalToVout(d), MIN_VOUT, MAX_VOUT, 0, 7), 0, 7);
}

int heightBar(int h) {
  return constrain(map(h, 0, 50, 0, 7), 0, 7);
}

char lcdbuf0[16];
char lcdbuf1[16];
// Guard
//char lcdbuf2[16] = "----++++----++++";

void lcdHelp() {
  char buf[16] = "0123456789ABCEF";
  char * messages[] = {
    "Options for help"
    "SELECT: Sim/Real",
    "LEFT  : Water   ",
    "RIGHT : Oil     ",
    "UP    : Help    ",
    "DOWN  : Uptime  "
  };
  for (int i = 0; i < 4; i++) {
    lcd.setCursor(0, 1);
    // TODO: cleanup to suppress compiler warning: deprecated conversion from string constant to 'char *'
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

void render_alarm(char * alarmtext, int status) {

  if (status == ALARM_DISABLED) {
    return;
  } else if (status == ALARM_ASSERT) {
    lcd.setCursor(0, 1);
    lcd.print(alarmtext);
    delay(3000);
    lcd.setCursor(0, 1);
    //lcd.print("0123456789012345");// Used to line up the 16 blanks below.
    lcd.print("                ");
    delay(1000);
  } else if (status == ALARM_FLASH) {
    for (int w = 0; w < 3; w++) {
      lcd.setCursor(0, 1);
      lcd.print(alarmtext);
      delay(1000);
      lcd.setCursor(0, 1);
      lcd.print("                ");
      delay(1000);
    }
  }
}

static float uts;
unsigned long tms;
unsigned long tms_prev;
unsigned long uptimeDD;
unsigned long uptimeHH;
unsigned long uptimeMM;
unsigned long residual;
unsigned long elapsed;

#define ACTION_NONE 0
#define ACTION_HELP 314
#define ACTION_UPTIME 271

void lcd_interface(int v_adc, unsigned long loops, int button_action) {

  int h = digitalToLevelCentimeters(v_adc);
  lcd.setCursor(0, 0);
  if (simulation) {
    sprintf(lcdbuf0, "%s%3d", simTitle, h);
  } else {
    if (liquidMode == LIQUID_WATER) {
    sprintf(lcdbuf0, "%s%3d", topTitleWater, h);
    } else if (liquidMode == LIQUID_JET_OIL) {
    sprintf(lcdbuf0, "%s%3d", topTitleJetOil, h);
      
    }
  }
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
  //sprintf(lcdbuf1, "%02d-%02d  Av %02d", hlo, hhi, hav);
  sprintf(lcdbuf1, "Average  :%3d", hav);
  lcd.setCursor(0, 1);
  lcd.print(lcdbuf1);
  int avbar = heightBar(hav);
  lcd.setCursor(13, 1);
  lcd.write((byte)avbar);

  sprintf(serbuf, "Analogue value %d Bar %d Loop %d", v_adc, avbar, loops);
  Serial.println(serbuf);

  if (simulation) {
    tms = millis() + ((86400 * 2) + (3600 * 15) + (42 * 60)) * 1000;
  } else {
    tms = millis();
  }

  if (tms > tms_prev) {
    uts += (tms - tms_prev) / 1000.;
  } else {
    uts += (tms_prev - tms) / 1000.;
  }
  elapsed = (unsigned long) uts;
  uptimeDD = (elapsed / 86400);
  residual = elapsed - (uptimeDD * 86400);
  uptimeHH = (residual / 3600);
  residual = residual - (uptimeHH * 3600);
  uptimeMM = (residual / 60);

  tms_prev = tms;

  sprintf(serbuf, "Uptime %ld %ld %02d:%02d:%02d", tms, elapsed, (int)uptimeDD, (int)uptimeHH, (int)uptimeMM);
  Serial.println(serbuf);

  sprintf(serbuf, "Uptime %ld %ld %ld", uptimeDD, uptimeHH, uptimeMM);
  Serial.println(serbuf);

  sprintf(serbuf, "lcd_update complete");
  Serial.println(serbuf);

}

#endif // LCD_IF_ENABLED

int32_t t_start;
volatile int32_t t_last;
int32_t cycle_t;

void setup_gem() {
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
    delay(2000);
    lcd.setCursor(0, 1);
    sprintf(lcdbuf1,  "Version : %s", release);
    lcd.print(lcdbuf1);
    delay(20000);
    /*
        for (int i = 0; i <= 16; i++) {
          lcd.scrollDisplayLeft();
          delay(SCROLL_DELAY);
        }
        for (int i = 0; i <= 16; i++) {
          lcd.scrollDisplayRight();
          delay(SCROLL_DELAY);
        }
        delay(1000);
    */
    lcd.clear();
  } // lcd_interface_enabled
#endif

  // start the Ethernet and UDP:
#if UDP_IF_ENABLED1
  Ethernet.begin(mac, ip, gw, nm);
  Udp.begin(localPort);
#endif

  serialHelp();

  //Get trigger levels and alarm configuration from EEPROM


  tLoLo = readInt16 (EEPROM_OFFSET_TLOLO);
  tLo = readInt16 (EEPROM_OFFSET_TLO);
  tHi = readInt16 (EEPROM_OFFSET_THI);
  tHiHi = readInt16(EEPROM_OFFSET_THIHI);
  delay_time = readInt32(EEPROM_OFFSET_DELAY);
  loloEnabled = readInt16(EEPROM_OFFSET_ELOLO);
  loEnabled = readInt16(EEPROM_OFFSET_ELO);
  hiEnabled = readInt16(EEPROM_OFFSET_EHI);
  hihiEnabled = readInt16(EEPROM_OFFSET_EHIHI);

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
    lcd.print("GEM Level Meter");
    lcd.setCursor(0, 1);
    lcd.print("Initialising HW");
    delay(2000);
    lcd.setCursor(0, 1);
    sprintf(lcdbuf1,  "Version : %s", release);
    lcd.print(lcdbuf1);
    delay(2000);
    lcd.clear();
  } // lcd_interface_enabled
#endif
  //

  //serialHelp();

  //Get trigger levels and alarm configuration from EEPROM

  /*
    tLoLo = readInt16 (EEPROM_OFFSET_TLOLO);
    tLo = readInt16 (EEPROM_OFFSET_TLO);
    tHi = readInt16 (EEPROM_OFFSET_THI);
    tHiHi = readInt16(EEPROM_OFFSET_THIHI);
    delay_time = readInt32(EEPROM_OFFSET_DELAY);
    loloEnabled = readInt16(EEPROM_OFFSET_ELOLO);
    loEnabled = readInt16(EEPROM_OFFSET_ELO);
    hiEnabled = readInt16(EEPROM_OFFSET_EHI);
    hihiEnabled = readInt16(EEPROM_OFFSET_EHIHI);
  */


  if (delay_time < 1000) {
    delay_time = 1000;
  }

  serialBuf[0] = '\0';

  loops = 0;

} // setup

int v_adc;
float sim_adc;
int h_cm;

#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

int adc_key_in;

int read_LCD_buttons(){               // read the buttons
    adc_key_in = analogRead(0);       // read the value from the sensor 

    // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
    // we add approx 50 to those values and check to see if we are close
    // We make this the 1st option for speed reasons since it will be the most likely result

    if (adc_key_in > 1000) return btnNONE; 

   // For V1.0 comment the other threshold and use the one below:
   
     if (adc_key_in < 50)   return btnRIGHT;  
     if (adc_key_in < 195)  return btnUP; 
     if (adc_key_in < 380)  return btnDOWN; 
     if (adc_key_in < 555)  return btnLEFT; 
     if (adc_key_in < 790)  return btnSELECT;     

    return btnNONE;                // when all others fail, return this.
}

void loop() {

  loops++;

  v_adc = analogRead (A1);

  int button = read_LCD_buttons();

  int button_action = ACTION_NONE;

  if (button == btnSELECT) {
     simulation = 1 - simulation;
  } else if (button == btnLEFT) {
  //    liquidMode = LIQUID_WATER;
  } else if (button == btnRIGHT) {
  //    liquidMode = LIQUID_JET_OIL;
  } else if (button == btnUP) {
    button_action = ACTION_HELP;
  } else if (button == btnDOWN) {
    button_action = ACTION_UPTIME;
  }
  
  if (simulation) {
    // In cm -> m
    sim_adc = (loops % LEVEL_CM_MAX_THRESHOLD) * 0.01;
    // In kpa
    if (liquidMode == LIQUID_WATER) {
      sim_adc = (sim_adc * G_ACC * RHO_WATER) / 1000.;
    } else if (liquidMode == LIQUID_JET_OIL) {
      sim_adc = (sim_adc * G_ACC * RHO_JET_OIL) / 1000.;
    }
    // As v_out
    sim_adc = kpaToVout(sim_adc);
    // As digital
    v_adc = voutToDigital(sim_adc);
  }

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


  if (lcd_interface_enabled) {
    lcd_interface(v_adc, loops, button_action);
  }

  if (Serial.available()) {
    //Serial.println("Parse serial input next");
    parseSerial();
  }

  delay (delay_time);
}


