#include <Arduino.h>
#include <Filter.h>
#include <EEPROM.h> // Settings storage based on https://gitlab.com/snippets/1728275

struct eeprom_config {
  int fullyClosed;
  int fullyOpened;
  int fullyOn;
  int fullyOff;
  int hysH;
  int hysL;
  long pvSmoothWeight;
  long spSmoothWeight;
  int ver;
  unsigned int checksum;
};

ExponentialFilter<long> pvSmooth(80,0);
ExponentialFilter<long> spSmooth(80,0);

const int pinSP     = 0;
const int pinPV     = 1;
const int pinMCl    = 6; // motor close signal
const int pinMOp    = 7; // motor open signal
const int pinOff    = 24; // NC switch so break = off
const int pinMenu   = 46;
const int pinSelect = 47;
const byte cmd = 0xFE; // LCD escape character

// parameters
eeprom_config settings; // setting struct
long pctSP;
long pctPV;
//bool buttonState = false;
//bool lastButtonState = false;
bool cmdOp = false;
bool cmdCl = false;
bool isOff = true; // From on/off DPDT switch that disconnects open signal
int valSP = 0;
int valPV = 0;
bool configError = false;

void lcdPower(bool power) {
  Serial3.write(cmd);
  if (power) { Serial3.write(0x41); }
  else { Serial3.write(0x42); }
}
bool lcdPos(byte position) {
  // 1: 00-13, 2: 40-53, 3: 14-27, 4: 54-67
  if ((position >= 0x00 && position <= 0x27) || (position >=0x40 && position <= 0x57)) {
    Serial3.write(cmd);
    Serial3.write(0x45);
    Serial3.write(position);
    return true;
  } else {
    return false;
  }
}
void lcdUnderline(bool underline) {
  Serial3.write(cmd);
  if (underline) { Serial3.write(0x47); }
  else { Serial3.write(0x48); }
}
void lcdLeft() {
  Serial3.write(cmd);
  Serial3.write(0x49);
}
void lcdRight() {
  Serial3.write(cmd);
  Serial3.write(0x4A);
}
void lcdBlink(bool blink) {
  Serial3.write(cmd);
  if (blink) { Serial3.write(0x4B); }
  else { Serial3.write(0x4C); }
}
void lcdBack() {
  Serial3.write(cmd);
  Serial3.write(0x4E);
}
void lcdClear() {
  Serial3.write(cmd);
  Serial3.write(0x51);
}
void lcdContrast(byte level){
  if (level < 0x01) level = 0x01;
  if (level > 0x50) level = 0x50;
  Serial3.write(cmd);
  Serial3.write(0x52);
  Serial3.write(level);
}
void lcdBrightness(byte level){
  if (level < 0x01) level = 0x01;
  if (level > 0x08) level = 0x08;
  Serial3.write(cmd);
  Serial3.write(0x53);
  Serial3.write(level);
}
void lcdCustom(byte addr, byte d0, byte d1, byte d2, byte d3, byte d4, byte d5, byte d6, byte d7 ) {
  byte character[] = {cmd, 0x54, addr, d0, d1, d2, d3, d4, d5, d6, d7};
  Serial3.write(character,sizeof(character));
}
void lcdShiftL() {
  Serial3.write(cmd);
  Serial3.write(0x55);
}
void lcdShiftR() {
  Serial3.write(cmd);
  Serial3.write(0x56);
}
void lcdDraw(bool op, bool cl, int pv, int sp, bool enabled) {
  // 1: 00-13, 2: 40-53, 3: 14-27, 4: 54-67
  if (!enabled) { // draw giant OFF
    lcdPos(0x00);
    Serial3.write(0x04);
    Serial3.write(0xFF);
    Serial3.write(0xFF);
    Serial3.write(0x03);
    Serial3.write(0x04);
    Serial3.write(0xFF);
    Serial3.write(0xFF);
    Serial3.write(0x03);
    Serial3.write(0x04);
    Serial3.write(0xFF);
    Serial3.write(0xFF);
    Serial3.write(0x03);
    lcdPos(0x40);
    Serial3.write(0x04);
    Serial3.print("  ");
    Serial3.write(0x03);
    Serial3.write(0x04);
    Serial3.write(0x01);
    Serial3.write(0x01);
    Serial3.print(" ");
    Serial3.write(0x04);
    Serial3.write(0x01);
    Serial3.write(0x01);
    lcdPos(0x14);
    Serial3.write(0x04);
    Serial3.print("  ");
    Serial3.write(0x03);
    Serial3.write(0x04);
    Serial3.write(0x02);
    Serial3.write(0x02);
    Serial3.print(" ");
    Serial3.write(0x04);
    Serial3.write(0x02);
    Serial3.write(0x02);
    lcdPos(0x54);
    Serial3.write(0x04);
    Serial3.write(0xFF);
    Serial3.write(0xFF);
    Serial3.write(0x03);
    Serial3.write(0x04);
    Serial3.print("   ");
    Serial3.write(0x04);
  }
  else {          // draw arrows as directed
    if (op) {     // draw open arrows
      lcdPos(0x00); // 0,0
      Serial3.write(0x7F); // right
      Serial3.print("O");
      Serial3.write(0x7E); // left
    }
    else {        // erase open arrows
      lcdPos(0x03);
      lcdBack(); lcdBack(); lcdBack();
    }
    if (cl) {     // draw close arrows
      lcdPos(0x54); // 4,0
      Serial3.write(0x7E); // left
      Serial3.print("C");// Serial3.write(0x20); // space
      Serial3.write(0x7F); // right
    }
    else {        // erase close arrows
      lcdPos(0x57);
      lcdBack();
      lcdBack();
      lcdBack();
    }
  }

  lcdPos(0x4C); // 2,(end - 8)
  Serial3.print("PV: ");
  if(pv<10) { // single digit, two pads
    Serial3.print("  ");
  } else if (pv<100) { // double digit, one pad
    Serial3.print(" ");
  }
  Serial3.print(pv);
  //displayPos(0x53);
  Serial3.print("%");

  lcdPos(0x20); // 3,(end - 8)
  Serial3.print("SP: ");
  if(sp<10) { // single digit, two pads
    Serial3.print("  ");
  } else if (sp<100) { // double digit, one pad
    Serial3.print(" ");
  }
  Serial3.print(sp);
  //    displayPos(0x27);
  Serial3.write("%");
}

void defaultConfig() {
  settings.fullyClosed = 610;
  settings.fullyOpened = 105;
  settings.fullyOn     = 993;
  settings.fullyOff    = 53;
  settings.hysH        = 5;
  settings.hysL        = 5;
  settings.pvSmoothWeight = 20;
  settings.spSmoothWeight = 80;
  settings.ver         = 0;
}
void loadConfig() {
  Serial.print("Loading config: checksum ");
  settings.checksum = 0;
  unsigned int sum = 0;
  unsigned char t;
  for(unsigned int i=0; i<sizeof(settings); i++) {
    t = (unsigned char)EEPROM.read(i);
    *((char *)&settings + i) = t;
    if(i < sizeof(settings) - sizeof(settings.checksum)) {
      sum = sum + t;
    }
  }
  Serial.println(sum);
  if(settings.checksum != sum) {
    Serial.println("Checksum fail, loading default config");
    configError = true;
    defaultConfig();
  }
  Serial.print("Closed: \t");
  Serial.println(settings.fullyClosed);
  Serial.print("Opened: \t");
  Serial.println(settings.fullyOpened);
  Serial.print("On: \t\t");
  Serial.println(settings.fullyOn);
  Serial.print("Off: \t\t");
  Serial.println(settings.fullyOff);
  Serial.print("Hys H/L: \t");
  Serial.print(settings.hysH);
  Serial.print("/");
  Serial.println(settings.hysL);
  Serial.print("Weight SP/PV: \t");
  Serial.print(settings.spSmoothWeight);
  Serial.print("/");
  Serial.println(settings.pvSmoothWeight);
  Serial.print("checksum: \t");
  Serial.println(settings.checksum);

  pvSmooth.SetWeight(settings.pvSmoothWeight);
  spSmooth.SetWeight(settings.spSmoothWeight);
}
void saveConfig() {
  Serial.print("Saving config: checksum ");
  unsigned int sum = 0;
  unsigned char t;
  for(unsigned int i = 0; i < sizeof(eeprom_config); i++) {
    if(i == sizeof(settings) - sizeof(settings.checksum)) {
      settings.checksum = sum;
    }
    t = *((unsigned char*)&settings + i);
    if(i < sizeof(settings) - sizeof(settings.checksum)) {
      /* Don't checksum the checksum! */
      sum = sum + t;
    }
    EEPROM.write(i, t);
  }
  Serial.println(sum);
}

int scale(int current, int high, int low) {
    if (high < low) {
      if (current < high) { current = high; }
      else if (current > low) { current = low; }
      return 100 - (current - low)/float(high - low) * 100;
    } else {
      if (current > high) { current = high; }
      else if (current < low) { current = low; }
      return (current - low)/float(high - low) * 100;
    }

}

void setup() {
    Serial.begin(9600);
    Serial3.begin(9600);
    Serial3.println("Please wait...");
    Serial3.println("Loading");
    Serial.println("Initializing I/O...");
    pinMode(pinMOp, OUTPUT);
    pinMode(pinMCl, OUTPUT);
    digitalWrite(pinMOp, LOW);
    digitalWrite(pinMCl, LOW);
    //    pinMode(pinButton, INPUT);
    pinMode(pinMenu, INPUT_PULLUP);
    pinMode(pinSelect, INPUT_PULLUP);
    pinMode(pinPV, INPUT);
    pinMode(pinSP, INPUT);
    pinMode(pinOff, INPUT_PULLUP);
    pvSmooth.SetCurrent(analogRead(pinPV));
    spSmooth.SetCurrent(analogRead(pinSP));
    Serial.println("Initialize display");
    lcdBrightness(8);
    lcdContrast(45);
    lcdBlink(false);
    lcdCustom(0x01, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F); // lower
    lcdCustom(0x02, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00); // upper
    lcdCustom(0x03, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C); // left
    lcdCustom(0x04, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07); // right
    lcdClear();
    Serial.println("Load settings");
    //TODO loadConfig();
    defaultConfig();
    lcdClear();
    //if(configError) saveConfig();

    // Serial3.print("0"); // gibberish
    // Serial3.write(0x00);
    // Serial3.print("1");
    // Serial3.write(0x01);
    // Serial3.print("2");
    // Serial3.write(0x02);
    // Serial3.print("3");
    // Serial3.write(0x03);
    // Serial3.print("4");
    // Serial3.write(0x04);
    // delay(5000);
    // Serial.println("Begin looping");
}

void loop() {
    spSmooth.Filter(analogRead(pinSP));
    pvSmooth.Filter(analogRead(pinPV));
    pctSP = scale(spSmooth.Current(),settings.fullyOff,settings.fullyOn);
    //pctPV = settings.ver;
    pctPV = 100 - scale(pvSmooth.Current(),settings.fullyOpened,settings.fullyClosed);
    // Serial.print("SP: ");
    // Serial.print(spSmooth.Current());
    // Serial.print(" = ");
    // Serial.print(pctSP);
    // Serial.print(". PV: ");
    // Serial.print(pvSmooth.Current());
    // Serial.print(" = ");
    // Serial.println(pctPV);
//    if(digitalRead(pinButton)==HIGH) {
    //  pctPV = pctSP;
    //  settings.ver = pctSP;
    //  saveConfig();
    //  lcdClear();
    //}
    //if (!cmdOp && )
    if (pctSP + settings.hysH <= pctPV) { // Valve too far open beyond hyst
      cmdOp = false;
      cmdCl = true;
      // Serial.println(". ==>Close<==");
    } else if (pctSP - settings.hysL >= pctPV) { // Valve too far closed
      cmdOp = true;
      cmdCl = false;
      // Serial.println(". <==Open==>");
    } else {
      cmdOp = false;
      cmdCl = false;
      // Serial.println(". <>HOLD<>");
    }

    if (!isOff && digitalRead(pinOff)) { // switch has changed to OFF
      isOff = true;
    } else if (isOff && !digitalRead(pinOff)) { // switch changed to ON
      isOff = false;
      lcdClear();
    }

    // if (cmdOp) { digitalWrite(pinMOp, LOW); }
      // else { digitalWrite(pinMOp, HIGH); }
    // if (cmdCl) { digitalWrite(pinMCl, LOW); }
      // else { digitalWrite(pinMCl, HIGH); }
    digitalWrite(pinMOp, cmdOp);
    digitalWrite(pinMCl, cmdCl);
    lcdDraw(cmdOp, cmdCl, pctPV, pctSP, !isOff);
    // delay(250);
}
