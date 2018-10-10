#include <Arduino.H>

const int buttonPin = 53;
const int knobPin = 0;
const int ledPin = 13;
const int openPin = 22;
const int closePin = 23;
const byte cmd = 0xFE; // pre-command control character

bool buttonState = false;
bool lastButtonState = false;
bool cmdOp = false;
bool cmdCl = false;
byte digit = 0x00;
int  knobSP = 0;
int  knobValue = 0;

void displayPower(bool power) {
  Serial3.write(cmd);
  if (power) { Serial3.write(0x41); }
  else { Serial3.write(0x42); }
}

bool displayPos(byte position) {
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

void displayUnderline(bool underline) {
  Serial3.write(cmd);
  if (underline) { Serial3.write(0x47); }
  else { Serial3.write(0x48); }
}

void displayLeft() {
  Serial3.write(cmd);
  Serial3.write(0x49);
}

void displayRight() {
  Serial3.write(cmd);
  Serial3.write(0x4A);
}

void displayBlink(bool blink) {
  Serial3.write(cmd);
  if (blink) { Serial3.write(0x4B); }
  else { Serial3.write(0x4C); }
}
void displayBack() {
  Serial3.write(cmd);
  Serial3.write(0x4E);
}

void displayCls() {
  Serial3.write(cmd);
  Serial3.write(0x51);
}

void displayContrast(byte level){
  if (level < 0x01) level = 0x01;
  if (level > 0x50) level = 0x50;
  Serial3.write(cmd);
  Serial3.write(0x52);
  Serial3.write(level);
}

void displayBrightness(byte level){
  if (level < 0x01) level = 0x01;
  if (level > 0x08) level = 0x08;
  Serial3.write(cmd);
  Serial3.write(0x53);
  Serial3.write(level);
}

void displayLshift() {
  Serial3.write(cmd);
  Serial3.write(0x55);
}

void displayRshift() {
  Serial3.write(cmd);
  Serial3.write(0x56);
}

void displayDraw(bool op, bool cl, int pv, int sp) {
    // 1: 00-13, 2: 40-53, 3: 14-27, 4: 54-67
    displayCls();
    if (op) {
      displayPos(0x00); // 0,0
      Serial3.write(0x7E); // right
      Serial3.write(0x20); // space
      Serial3.write(0x7F); // left
    }

    if (cl) {
      displayPos(0x54); // 4,0
      Serial3.write(0x7F); // left
      Serial3.write(0x20); // space
      Serial3.write(0x7E); // right
    }

    displayPos(0x4C); // 2,(20-8)
    Serial3.print("PV: ");
    if(pv<10) { // single digit, two pads
      Serial3.print("  ");
    } else if (pv<100) { // double digit, one pad
      Serial3.print(" ");
    }
    Serial3.print(pv);
    //displayPos(0x53);
    Serial3.print("%");

    displayPos(0x20); // 3,(20-8)
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial3.begin(9600);
  displayBrightness(8);
  displayContrast(45);
  displayBlink(false);
  Serial.println("Starting...");
  pinMode(buttonPin, INPUT);
  pinMode(knobPin, INPUT);
  pinMode(ledPin, OUTPUT);
  pinMode(openPin, OUTPUT);
  pinMode(closePin, OUTPUT);
}


void loop() {
  knobValue = analogRead(knobPin);
  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    digitalWrite(ledPin, HIGH);
    knobSP = knobValue;
  } else {
    digitalWrite(ledPin, LOW);
  }

  int pvpct = knobValue/10.24;
  int sppct = knobSP/10.24;
  if (pvpct > sppct) {
    cmdOp = false;
    cmdCl = true;
  } else if (pvpct < sppct) {
    cmdOp = true;
    cmdCl = false;
  } else {
    cmdOp = false;
    cmdCl = false;
  }
  displayDraw(cmdCl, cmdOp, pvpct, sppct);
  digitalWrite(openPin, !cmdOp); // LOW to light
  digitalWrite(closePin, !cmdCl); // LOW to light
  delay(100);
//  lastButtonState = buttonState;
}
