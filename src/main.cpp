#include <Arduino.h>
#include <Filter.h>
#include <EEPROM.h> // Settings storage based on https://gitlab.com/snippets/1728275
#include <PID_v1.h>

#define PIN_PV          0
#define PIN_SP          1
#define PIN_MCL         6
#define PIN_MOP         7
#define PIN_SELECT      23
#define PIN_MENU        25
#define PIN_OFF         27
#define CMD           0xFE
#define LONG_PRESS      1750
#define LONGLONG_PRESS  4250

enum menus {
  PV_CAL,     //  0
  PV_CONFIRM,
  PV_OPENING,
  PV_CLOSING,
  PV_SAVE,
  SP_CAL,     //  5
  SP_OPENING,
  SP_CLOSING,
  SP_SAVE,
  CLOSED_SP,  //  9
  CLOSED_CONFIRM,
  MENU_END    //  11
};
struct eeprom_config {
  int fullyClosed;
  int fullyOpened;
  int fullyOn;
  int fullyOff;
  int hysH;
  int hysL;
  int pvSmoothWeight;
  int spSmoothWeight;
  int ver;
  unsigned int checksum;
};

// parameters
ExponentialFilter<long> pvSmooth(80,0);
ExponentialFilter<long> spSmooth(80,0);

eeprom_config settings;
int valSP = 0;      // SP pot raw value
double pctSP = 0;      // SP scaled to %
int valPV = 0;      // PV pot raw value
double pctPV = 0;      // PV scaled to %
bool cmdOp = false; // send Open signal
bool cmdCl = false; // send Close signal
bool isOff = true;  // Main toggle switch PIN_OFF
bool configError = true;// error loading from EEPROM
bool menuMode = false;  //
menus menuPage;     // Current menu page for button logic
int menuButton = 0; // button status 0=up, 1=short, 2=long
int selButton = 0;  // button status 0=up, 1=short, 2=long
unsigned int shortPress = 100;             // debounce time ms
unsigned int longPressM = LONG_PRESS;      // Menu button long press ms
unsigned int longPressS = LONGLONG_PRESS;  // Sel button long press ms
bool displayChange = true;        // flag to trigger redraw

// PID additions
double pidProportion = 0;
double Kp =0.5, Ki=0.5, Kd=0;
double pvError = 0;
PID gatePID(&pctPV, &pidProportion, &pctSP, Kp, Ki, Kd, P_ON_M, DIRECT);
unsigned int windowSize = 2500;      // How long of a PTC window. Op/Cl ratio will be of this time
unsigned long windowStart;  // Start time of current PTC window
unsigned long loopStart;    // DEBUG
bool pidMode = MANUAL;

void defaultConfig() {
  settings.fullyClosed = 610;
  settings.fullyOpened = 105;
  settings.fullyOn     = 993;
  settings.fullyOff    = 53;
  settings.hysH        = 1;
  settings.hysL        = 1;
  settings.pvSmoothWeight = 60;
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
  Serial.print("Closed: \t");   Serial.print(settings.fullyClosed);
  Serial.print("\tOpened: \t"); Serial.println(settings.fullyOpened);
  Serial.print("On: \t\t");     Serial.print(settings.fullyOn);
  Serial.print("Off: \t\t");    Serial.println(settings.fullyOff);
  Serial.print("Hys H/L: \t");  Serial.print(settings.hysH);
  Serial.print("/");            Serial.println(settings.hysL);
  Serial.print("Weight SP/PV\t"); Serial.print(settings.spSmoothWeight);
  Serial.print("/");            Serial.println(settings.pvSmoothWeight);
  pvSmooth.SetWeight(settings.pvSmoothWeight);
  spSmooth.SetWeight(settings.spSmoothWeight);
}
void saveConfig() {
  Serial.print("Saving config. checksum: ");
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
  // TODO: Verify save
  configError = false;
}

/* LCD Control functions
    lcdPos positions = 1: 00-13, 2: 40-53, 3: 14-27, 4: 54-67
    lcdContrast 1..50
    lcdBrightness 1..8 */
void lcdPower(bool power) {
  Serial3.write(CMD);
  if (power) { Serial3.write(0x41); }
  else { Serial3.write(0x42); }
}
bool lcdPos(byte position) {
  // 1: 00-13, 2: 40-53, 3: 14-27, 4: 54-67
  if ((position >= 0x00 && position <= 0x27) || (position >=0x40 && position <= 0x57)) {
    Serial3.write(CMD);
    Serial3.write(0x45);
    Serial3.write(position);
    return true;
  } else {
    return false;
  }
}
void lcdUnderline(bool underline) {
  Serial3.write(CMD);
  if (underline) { Serial3.write(0x47); }
  else { Serial3.write(0x48); }
}
void lcdLeft() {
  Serial3.write(CMD);
  Serial3.write(0x49);
}
void lcdRight() {
  Serial3.write(CMD);
  Serial3.write(0x4A);
}
void lcdBlink(bool blink) {
  Serial3.write(CMD);
  if (blink) { Serial3.write(0x4B); }
  else { Serial3.write(0x4C); }
}
void lcdBack() {
  Serial3.write(CMD);
  Serial3.write(0x4E);
}
void lcdClear() {
  Serial3.write(CMD);
  Serial3.write(0x51);
}
void lcdContrast(byte level){
  if (level < 0x01) level = 0x01;
  if (level > 0x50) level = 0x50;
  Serial3.write(CMD);
  Serial3.write(0x52);
  Serial3.write(level);
}
void lcdBrightness(byte level){
  if (level < 0x01) level = 0x01;
  if (level > 0x08) level = 0x08;
  Serial3.write(CMD);
  Serial3.write(0x53);
  Serial3.write(level);
}
void lcdCustom(byte addr, byte d0, byte d1, byte d2, byte d3, byte d4, byte d5, byte d6, byte d7 ) {
  byte character[] = {CMD, 0x54, addr, d0, d1, d2, d3, d4, d5, d6, d7};
  Serial3.write(character,sizeof(character));
}
void lcdShiftL() {
  Serial3.write(CMD);
  Serial3.write(0x55);
}
void lcdShiftR() {
  Serial3.write(CMD);
  Serial3.write(0x56);
}
void lcdDraw(bool op, bool cl, int pv, int sp, bool enabled) {
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
  else {      // draw arrows as needed
    if(op) {  //  open
      lcdPos(0x00); // row1
      Serial3.write(0x7F); // <
      Serial3.print("O");
      Serial3.write(0x7E); // >
    }
    else {    //  no open
      lcdPos(0x03);
      lcdBack(); lcdBack(); lcdBack();
    }
    if (cl) { // close
      lcdPos(0x54); // row 4
      Serial3.write(0x7E);  // >
      Serial3.print("C");
      Serial3.write(0x7F);  // <
    }
    else {    // no close
      lcdPos(0x57);
      lcdBack(); lcdBack(); lcdBack();
    }
  }

  lcdPos(0x4C); // row 2, 8 from end
  Serial3.print("PV: ");
  if(pv<10) { // single digit, two pads
    Serial3.print("  ");
  } else if (pv<100) { // double digit, one pad
    Serial3.print(" ");
  }
  Serial3.print(pv);
  Serial3.print("%");

  lcdPos(0x20); // row 3, 8 from end
  Serial3.print("SP: ");
  if(sp<10) {
    Serial3.print("  ");
  } else if (sp<100) {
    Serial3.print(" ");
  }
  Serial3.print(sp);
  Serial3.print("%");
}

/* button polling functions - short, long, debounce */
int checkMenuButton() {
  const int PIN = PIN_MENU;
  static bool buttonState = HIGH;
  static bool prevState = HIGH;
  static bool fired = false;
  static unsigned long pressTime;

  buttonState = digitalRead(PIN);

  if(buttonState == LOW) {                // pressed
    if(prevState == HIGH) {               //  just now
      prevState = LOW;                    //    set as pressed
      pressTime = millis();               //    and mark the time
      return 0;
    } else {                              //  in the past & being held
      if((millis() - pressTime > longPressM) && !fired) { // time is up and we haven't fired yet
        fired = true;                     //      set fired flag to prevent repeat
        Serial.print("menL");
        return 2;                         //      return long signal
      }
      return 0;                           //    time's not up, carry on
    }
  } else {                                // not pressed
    if(prevState == LOW) {                //   and it was pressed before
      prevState = HIGH;                   //     clear pressed flag
      if((millis() - pressTime > shortPress) && !fired) { // short time had passed but not long
        Serial.print("menS");
        return 1;                         //       return short signal
      }
      return 0;
    }
    fired = false;                        //   clear fired flag for next time
    return 0;
  }
  return -1;
}
int checkSelButton() {
  const int PIN = PIN_SELECT;
  static bool buttonState = HIGH;
  static bool prevState = HIGH;
  static bool fired = false;
  static unsigned long pressTime;

  buttonState = digitalRead(PIN);

  if(buttonState == LOW) {                // pressed
    if(prevState == HIGH) {               //  just now
      prevState = LOW;                    //    set as pressed
      pressTime = millis();               //    and mark the time
      return 0;
    } else {                              //  in the past & being held
      if((millis() - pressTime > longPressS) && !fired) { // time is up and we haven't fired yet, now w/ rollover protection
        fired = true;                     //      set fired flag to prevent repeat
        Serial.print("selL");
        return 2;                         //      return long signal
      }
      return 0;                           //    time's not up, carry on
    }
  } else {                                // not pressed
    if(prevState == LOW) {                //   and it was pressed before
      prevState = HIGH;                   //     clear pressed flag
      if((millis() - pressTime > shortPress) && !fired) { // short time had passed but not long, now w/ rollover protection
        Serial.print("selS");
        return 1;                         //       return short signal
      }
      return 0;
    }
    fired = false;                        //   clear fired flag for next time
    return 0;
  }
  return -1;
}

/* Menu display & traversal */
void lcdPrint(const char* line1, const char* line2, const char* line3, const char* line4) {
  lcdClear();
  lcdPos(0x00);
  Serial3.print(line1);
  lcdPos(0x40);
  Serial3.print(line2);
  lcdPos(0x14);
  Serial3.print(line3);
  lcdPos(0x54);
  Serial3.print(line4);
}
void menuUpdate(bool newmenu) {
  static int new100 = 0, new0 = 0;
  if(newmenu) { // set up initial menus & exit without processing
    lcdPrint("  Calibrate gate?   ","   DO NOT RUN WITH  ","   FULL GRIST CASE! ","YES               NO");
    longPressM = LONGLONG_PRESS;
    menuPage = PV_CAL;
    return;
  }
  if(menuButton == 2) { // long press during menumode, end mode & exit
    Serial.println("end menu mode");
    lcdClear();
    longPressM = LONG_PRESS;
    longPressS = LONG_PRESS;
    menuMode = false;
    return;
  }
  switch(menuPage) { // process buttons from menuPage
    case PV_CAL:
      if (menuButton == 1) {        // menu short press
        menuPage = SP_CAL;          //   next menu
      } else if (selButton == 1) {  // sel short press
          menuPage = PV_CONFIRM;    //   go to confirm
          longPressS = LONGLONG_PRESS; // confirm should hold out for longlong press!
      }
    break;
    case PV_CONFIRM:
      if (menuButton == 1) {        // menu short press
        menuPage = PV_CAL;          //   back to prompt
      } else if (selButton == 2) {  // sel verylong press
          menuPage = PV_OPENING;    //   go to opening
          Serial.println("TODO: open relay on"); // begin opening gate
      }
    break;
    case PV_OPENING:
      if (selButton == 1) {         // sel short press
        menuPage = PV_CLOSING;      //   go to close
        Serial.println("TODO: open relay off"); // stop opening gate
        delay(250);                 // give pot a moment to settle
        new100   = analogRead(PIN_PV); //store full open value
        Serial.println("TODO: close relay on"); // start closing gate
      }
    break;
    case PV_CLOSING:
      if (selButton == 1) {         // sel short press
        menuPage = PV_SAVE;         //   go to save
        Serial.println("TODO: close relay off"); // stop closing gate
        delay(250);                 // give pot a moment to settle
        new0   = analogRead(PIN_PV); //  store full close value
      }
    break;
    case PV_SAVE:
      if (menuButton == 1) {        // menu short press
        menuPage = PV_CAL;          //   go back to start
      } else if (selButton == 2) {  // sel verylong press
        Serial.print("PV: "); Serial.print(new0); Serial.print(" => "); Serial.println(new100);
        settings.fullyOpened = new100; //   store settings and save to EEPROM
        settings.fullyClosed = new0;
        saveConfig();
        menuPage = SP_CAL;          //   on to next setting
      }
    break;
    case SP_CAL:
      if (menuButton == 1) {        // menu short press
        menuPage = CLOSED_SP;       //   next menu
      } else if (selButton == 1) {  // sel short press
          menuPage = SP_OPENING;    //   begin calibration
      }
    break;
    case SP_OPENING:
      if (menuButton == 1) {        // menu short press
        menuPage = SP_CAL;          //   go back
      } else if (selButton == 1) {  // sel short press
        menuPage = SP_CLOSING;      //   next step
        new100   = analogRead(PIN_SP); //store full open value
      }
    break;
    case SP_CLOSING:
      if (menuButton  == 1) {       // menu short press
        menuPage = SP_CAL;          //   go back
      } else if (selButton == 1) {  // sel short press
        menuPage = SP_SAVE;         //   next step
        new0   = analogRead(PIN_SP);//   store full close value
      }
    break;
    case SP_SAVE:
      if (menuButton == 1) {        // menu short press
        menuPage = SP_CAL;          //   go back to start
      } else if (selButton == 2) {  // sel verylong press
        Serial.print("SP: "); Serial.print(new0); Serial.print(" => "); Serial.println(new100);
        settings.fullyOn = new100; //   store settings and save to EEPROM
        settings.fullyOff = new0;
        // save settings and show a confirmation screen
        saveConfig();
        menuPage = CLOSED_SP;       //  on to next setting
      }
    break;
    case CLOSED_SP:
      if (menuButton == 1) {        // menu short press
        menuPage = MENU_END;        //   go to next item
      } else if (selButton == 1) {  // sel short press
        menuPage = CLOSED_CONFIRM;  //   go to next step
      }
    break;
    case CLOSED_CONFIRM:
      if (menuButton == 1) {        // menu short press
        menuPage = CLOSED_SP;       //  go back
      } else if (selButton == 2) {  // sel verylong press
        Serial.print("Saving new Closed");
        // settings.closed = PV; //   store settings and save to EEPROM
        // save settings and show a confirmation screen
       menuPage = MENU_END;        //  next item
      }
    break;
    case MENU_END:
      if (menuButton == 1) {        // menu press short
        menuPage = PV_CAL;          //  back to the beginning
      } else if (selButton == 1) {  // sel press short
        menuMode = false;           //  end menu mode
      }
    break;
  }
  switch(menuPage) { // draw the new menuPage before quitting.
    case PV_CAL:
      lcdPrint("  Calibrate gate?   ","   DO NOT RUN WITH  ","   FULL GRIST CASE! ","YES               NO"); break;
    case PV_CONFIRM:
      lcdPrint(" Switch must be on  ","   GATE WILL OPEN!  ","Hold SEL to confirm ","CONFIRM (5s)    BACK"); break;
    case PV_OPENING:
      lcdPrint("   OPENING GATE     ","Press SEL once open ","                    ","OPENED              "); break;
    case PV_CLOSING:
      lcdPrint("    CLOSING GATE    ","Press SEL once closd","                    ","CLOSED              "); break;
    case PV_SAVE:
      lcdPrint("Save new CL-OP vals?"," Old: #### => ####  ","  New: #### => #### ","SAVE (5s)     CANCEL"); break;  // TODO: fill in values
    case SP_CAL:
      lcdPrint("  Calibrate knob %  "," Safe to run. Gate  "," will not be moved. ","YES               NO"); break;
    case SP_OPENING:
      lcdPrint("    TURN KNOB =>    ","   all the way on   ","Press SEL when done.","FULL ON       CANCEL"); break;
    case SP_CLOSING:
      lcdPrint("    TURN KNOB <=    ","   all the way off  ","Press SEL when done.","FULL OFF      CANCEL"); break;
    case SP_SAVE:
      lcdPrint("  Save new limits?  "," Old: #### => ####  ","  New: #### => #### ","SAVE (5s)     CANCEL"); break; // TODO: fill in values
    case CLOSED_SP:
      lcdPrint(" Set closed PV %    ","  Call current pos  "," 'closed' in Braumat","YES               NO"); break;
    case CLOSED_CONFIRM:
      lcdPrint("Current PV ###%     ","Tell Braumat this   ","is \"CLOSED\" limit?  ","SAVE (5s)     CANCEL"); break; // TODO: fill in values
    case MENU_END:
      lcdPrint("Done making changes?","                    ","                    ","EXIT      START OVER"); break;
  }
  if(menuMode == false) {
    lcdClear();
  }
}

void setup() {
    // Initialize serials
    Serial.begin(9600);
    Serial.println("Starting up");
    Serial3.begin(9600);
    lcdBlink(true);
    lcdPos(0x00); Serial3.print(" Starting, pls wait");
    lcdPos(0x00);
    // initialize pins
    pinMode(PIN_MENU, INPUT_PULLUP);
    pinMode(PIN_SELECT, INPUT_PULLUP);
    pinMode(PIN_OFF, INPUT_PULLUP);
    pinMode(PIN_MOP, OUTPUT);   // off immediately
    pinMode(PIN_MCL, OUTPUT);   // off immediately
    digitalWrite(PIN_MOP, LOW);
    digitalWrite(PIN_MCL, LOW);
    pinMode(PIN_PV, INPUT);
    pinMode(PIN_SP, INPUT);

    // initialize filters
    pvSmooth.SetCurrent(analogRead(PIN_PV));
    spSmooth.SetCurrent(analogRead(PIN_SP));

    // initialize display (custom chars)
    lcdPos(0x40); Serial3.print("  Customizing LCD");
    lcdPos(0x40);
    lcdBrightness(8);
    lcdContrast(45);
    lcdBlink(false);
    lcdCustom(0x01, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F); // lower
    lcdCustom(0x02, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x00, 0x00, 0x00); // upper
    lcdCustom(0x03, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C); // left
    lcdCustom(0x04, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07); // right

    // load EEPROM configuration
    lcdPos(0x14); Serial3.print("  Loading settings");
    lcdPos(0x14);
    loadConfig();
    lcdClear();

    // activate PID
    gatePID.SetOutputLimits(-windowSize,windowSize);
    // gatePID.SetSampleTime(250); // default is already 100
    gatePID.SetMode(pidMode);

    // Ready to rock!
    windowStart = millis();
    Serial.print("Startup complete after ");
    Serial.print(windowStart);
    Serial.println("ms");
}

void loop() {
  // poll events
  menuButton = checkMenuButton();
  selButton = checkSelButton();
  spSmooth.Filter(analogRead(PIN_SP));
  pvSmooth.Filter(analogRead(PIN_PV));
  // Menu branches all lead to early return
  if (menuMode) {                           // In menu mode
    digitalWrite(PIN_MOP, LOW);             //   STOP MOVING
    digitalWrite(PIN_MCL, LOW);             //   STOP MOVING
    if (selButton > 0 || menuButton > 0) {  //   and a button has been pressed
      menuUpdate(false);                    //     update display
    }
    return;                                 // do no more processing
  } else if (menuButton == 2) {             // long menu press
    menuMode = true;                        //   enter menu mode
    menuUpdate(true);                       //   - first run
    return;                                 //   do no more processing
  }

  // only non-menu mode stuff below here
  pctSP = map(spSmooth.Current(),settings.fullyOff,settings.fullyOn,0,100);
  pctPV = map(pvSmooth.Current(),settings.fullyClosed,settings.fullyOpened,0,100);
  //pvError = abs(pctSP - pctPV);   // how far out of position are we either way?
  gatePID.Compute();
  Serial.print(pidProportion);
  // TODO get rid of H & L?
  // if(!cmdOp && !cmdCl && (pvError > settings.hysH)) { // holding out of position
  pidMode = gatePID.GetMode();
  if(pidMode==MANUAL && (pvError > settings.hysH)) { // stopped & out of position
    Serial.print("Enabling ");
    pidMode = AUTOMATIC;                             //  start moving
    gatePID.SetMode(pidMode);
    gatePID.Compute();
    // BAD BAD BAD TODO: direction can't change while PID is active
    // if (pctSP > pctPV) {                              //   in the right direction
    //   cmdOp = true; cmdCl = false;
    // } else if (pctSP < pctPV) {
    //   cmdCl = true; cmdOp = false;
    // }
  } else if((pidMode==AUTOMATIC) && (pidProportion < 125)) { // moving & less than one frame out of position
    Serial.println("PID off");
    pidMode = MANUAL;                           //  quit moving
    gatePID.SetMode(pidMode);
  } // not addressed: moving & out of position, stopped & in position

  if(pidMode == AUTOMATIC) {    // Still moving, cuz we would have stopped if in range
    if (pctSP > pctPV) {                              //   set direction
      cmdOp = true; cmdCl = false;
    } else if (pctSP < pctPV) {
      cmdCl = true; cmdOp = false;
    }
  } else {
    cmdOp = false; cmdCl = false;                      //  in either direction
    pidProportion = 0;
  }
  if(isOff) {         // override everything, turn PID back off
    pidMode = MANUAL;
    gatePID.SetMode(pidMode);
    digitalWrite(PIN_MOP, LOW);
    digitalWrite(PIN_MCL, LOW);
    cmdOp = false;
    cmdCl = false;
  }
// DEBUG
  // if (millis() - windowStart > 100) {
  //   Serial.println(millis()-windowStart);
  // }
  if(pidMode == AUTOMATIC) {
    gatePID.Compute();
    if (millis() - windowStart > windowSize) { //time to shift the Relay Window
      windowStart += windowSize;                 // shift window from end time, not now time
    }
    if (pidProportion < millis() - windowStart) {
      digitalWrite(PIN_MOP, HIGH && cmdOp);
      digitalWrite(PIN_MCL, HIGH && cmdCl);
    } else {
      digitalWrite(PIN_MCL, LOW);
      digitalWrite(PIN_MOP, LOW);
    }
  }

  if(!isOff && digitalRead(PIN_OFF)) { // switch changed to open
    isOff = true;
  } else if (isOff && !digitalRead(PIN_OFF)) { // switch changed closed
    isOff = false;
    lcdClear();
  }

  lcdDraw(cmdOp, cmdCl, pctPV, pctSP, !isOff);
  if((millis()-loopStart)>99) { Serial.println(millis()-loopStart); }
  loopStart = millis();
}
