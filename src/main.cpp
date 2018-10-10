#include <Arduino.H>
#include <Filter.h>

ExponentialFilter<long> pvSmooth(20,0);
ExponentialFilter<long> spSmooth(30,0);

const int pinButton = 53;
const int pinSP     = 0;
const int pinPV     = 1;
const int pinStatus = 13;
const int pinMOp    = 7; // motor open signal, relay1
const int pinMCl    = 6; // motor close signal, relay2
const int pinOff    = 24; // NC switch so break = off
const byte cmd = 0xFE; // LCD escape character

// parameters
int fullyClosed = 610; // levels as tested 9/10/2018
int fullyOpened = 105;
int fullyOn     = 993; // pot max/min readings
int fullyOff    = 53;
int hysH        = 2;
int hysL        = 2;

long pctSP;
long pctPV;
bool buttonState = false;
bool lastButtonState = false;
bool cmdOp = false;
bool cmdCl = false;
bool isOff = true; // From on/off DPDT switch that disconnects open signal
int valSP = 0;
int valPV = 0;


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
      Serial3.write(0x7E); // right
      Serial3.write(0x20); // space
      Serial3.write(0x7F); // left
    }
    else {        // erase open arrows
      lcdPos(0x03);
      lcdBack();
      lcdBack();
      lcdBack();
    }
    if (cl) {     // draw close arrows
      lcdPos(0x54); // 4,0
      Serial3.write(0x7F); // left
      Serial3.write(0x20); // space
      Serial3.write(0x7E); // right
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
    digitalWrite(pinMOp, HIGH);
    digitalWrite(pinMCl, HIGH);
    pinMode(pinButton, INPUT);
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
    //pvSmooth.Filter(analogRead(pinPV));
    spSmooth.Filter(analogRead(pinSP));
    pctSP = scale(spSmooth.Current(),fullyOff,fullyOn);
    pctPV = 68; // DEBUG scale(pvSmooth.Current(),fullyOpened,fullyClosed);
    Serial.print(spSmooth.Current());
    Serial.print(" = ");
    Serial.print(pctSP);
    Serial.print(". PV=");
    Serial.print(pctPV);

    if (pctSP + hysH <= pctPV) { // Valve too far open beyond hyst
      cmdOp = false;
      cmdCl = true;
      Serial.println(". ==>Close<==");
    } else if (pctSP - hysL >= pctPV) { // Valve too far closed
      cmdOp = true;
      cmdCl = false;
      Serial.println(". <==Open==>");
    } else {
      cmdOp = false;
      cmdCl = false;
      Serial.println(". <>HOLD<>");
    }
    if (!isOff && digitalRead(pinOff)) { // switch has changed to OFF
      isOff = true;
    } else if (isOff && !digitalRead(pinOff)) { // switch changed to ON
      isOff = false;
      lcdClear();
    }
    digitalWrite(pinMOp, !cmdOp);
    digitalWrite(pinMCl, !cmdCl);
    lcdDraw(cmdOp, cmdCl, pctPV, pctSP, !isOff);
    delay(250);
}
