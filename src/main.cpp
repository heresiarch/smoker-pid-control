#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <Adafruit_MAX31865.h>
#include <TimerFour.h>
#include <PID_v1.h>

#include "MyButton.h"
#include "RotaryEncoderSwitch.h"
#include "PlatinumSensor.h"
#include "bbqfan.h"


int pwmPin = 3;

LiquidCrystal lcd(7,8,9,10,11,12);
char line0[21]; 
char line1[21];

Adafruit_MAX31865 max = Adafruit_MAX31865(2,4,5,6);
RotaryEnoderSwitch rot = RotaryEnoderSwitch(A0,A1,EncoderType::twoStep);
MyButton mybutton = MyButton(A2,true,false);

enum operatingState { INIT = 0, OFF,SET_TEMP, SET_TIMER, MENU, MANUAL_MODE, PREP_FUZZY, RUN_FUZZY, PREP_PID, RUN_PID};
operatingState opState = OFF;

#define RIGHT_MOVE 0
#define LEFT_MOVE 1
#define BUTTON_PUSH 2
#define BUTTON_LONG_PUSH 3


uint16_t startMillis;  //some global variables available anywhere in the program
uint16_t currentMillis;
uint16_t period = 1000;  //the value is a number of milliseconds

uint16_t startMillisDisplay;  //some global variables available anywhere in the program
uint16_t currentMillisDisplay;
uint16_t periodDisplay = 5000;  //the value is a number of milliseconds
bool switchDisplay = false;

double currentTemp = 0.0;
double targetTemp = 60.0;
#define TARGET_TEMP_MAX 180.0
#define TARGET_TEMP_MIN 20.0
#define TIMER_MAX 840
#define TIMER_MIN 5
uint16_t targetTimer = TIMER_MIN;

uint8_t pwmValue = 0;
#define MAX_PWM  255
#define MIN_PWM  30

double outputVal;
double aggKp=8, aggKi=0.2, aggKd=1;
double consKp=2, consKi=0.05, consKd=0.25;

PID myPID(&currentTemp, &outputVal, &targetTemp, consKp, consKi, consKd, DIRECT);
BBQFan bbqfan(currentTemp, targetTemp, pwmValue);
unsigned long lastFuzzy = 0;
#define FUZZY_PERIOD_MS 5000
                                               //0123456789ABCDEF     
static const char OFF_LINE_00[] PROGMEM =       "BBQ is Off      ";
static const char OFF_LINE_01[] PROGMEM =       "Press Button    ";
static const char SET_T_LINE_00[] PROGMEM =     "Set Temperature ";
static const char SET_T_LINE_01[] PROGMEM =     "                ";
static const char SET_TIMER_LINE_00[] PROGMEM = "Set Timer       ";
static const char SET_TIMER_LINE_01[] PROGMEM = "    min         ";
static const char MENU_ITEMS [4][17] PROGMEM =  {" Manual Mode    ",
                                                 " PID Control    ",
                                                 " Fuzzy          ",     
                                                 " Reset          "};

static const char SET_MAN_LINE_00[] PROGMEM =    "Tact:          C";
static const char SET_MAN_LINE_01[] PROGMEM =    "Ttar:          C";
static const char SET_MAN_LINE_02[] PROGMEM =    "PWM value       ";
static const char SET_MAN_LINE_03[] PROGMEM =    "                ";

static const char SET_AUTO_LINE_00[] PROGMEM =    "               ";
static const char SET_AUTO_LINE_01[] PROGMEM =    "               ";


uint8_t menuIdx = 0;

void timer4(void)
{
  rot.tickDebounceDecode();
  mybutton.debounce();
}


void setup() {
  // PWM on Pin 3
  //http://www.scynd.de/tutorials/arduino-tutorials/3-luefter-steuern/3-1-pwm-ohne-pfeifen.html
  //TCCR1B = TCCR1B & 0b11111000 | 0x01;
  pinMode(pwmPin, OUTPUT);
  lcd.begin(16, 2);
	// Clears the LCD screen
	lcd.clear();
  Serial.begin(115200);
  max.begin(MAX31865_4WIRE);
  Timer4.initialize(1000);
  Timer4.attachInterrupt(timer4);
  /*
  Telemetry.attach_f32_to("p", &p);
  Telemetry.attach_f32_to("i", &i);
  Telemetry.attach_f32_to("d", &d);
  */
  opState = INIT;
}

float readTemperature(void){
  float rtd = (float)max.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  // Check and print any faults
  float temp = PlatinumSensor::tempFromPtResistance(4300.00*ratio,1000.00);
  uint8_t fault = max.readFault();
  if (fault) {
    Serial.print(F("Fault 0x")); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println(F("RTD High Threshold")); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println(F("RTD Low Threshold")); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println(F("REFIN- > 0.85 x Bias")); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println(F("REFIN- < 0.85 x Bias - FORCE- open")); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println(F("RTDIN- < 0.85 x Bias - FORCE- open")); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println(F("Under/Over voltage")); 
    }
    max.clearFault();
  }
  return temp;   
}

uint8_t readButtons()
{
  uint8_t buttons = 0;
  int8_t idx = rot.readEncoder();
  if(idx > 0)
      buttons |= (1 << LEFT_MOVE);
  else if(idx < 0)
      buttons |= (1 << RIGHT_MOVE);
  
  currentMillis = millis();
  if(mybutton.buttonPressed() && (currentMillis - startMillis >= period)){
    startMillis = currentMillis;
    if(mybutton.longPressed())
      buttons |= (1 << BUTTON_LONG_PUSH);
    else
      buttons |= (1 << BUTTON_PUSH);
  }
  return buttons;
}

void welcomeMsg(void){
  strncpy_P(line0, OFF_LINE_00, 16);
  strncpy_P(line1, OFF_LINE_01, 16);
}

void targetTempMsg(void){
  strncpy_P(line0, SET_T_LINE_00, 16);
  strncpy_P(line1, SET_T_LINE_01, 16);
  line1[4] = 0xDF;
  line1[5] = 'C';
  String value = String(targetTemp,0);
  strncpy(line1,value.c_str(),value.length());
}

void timerMsg(void){
  strncpy_P(line0, SET_TIMER_LINE_00, 16);
  strncpy_P(line1, SET_TIMER_LINE_01, 16);
  line1[4] = 'm';
  line1[5] = 'i';
  line1[6] = 'n';
  String value = String(targetTimer,10);
  strncpy(line1,value.c_str(),value.length());
}

void menuMsg(){ //TODO rework scrolling too many ifs
    if(menuIdx == 0){
      strncpy_P(line0, MENU_ITEMS[0], 16);
      strncpy_P(line1, MENU_ITEMS[1], 16);
      line0[0] = '>';  
    }
    else if(menuIdx == 1){
      strncpy_P(line0, MENU_ITEMS[0], 16);
      strncpy_P(line1, MENU_ITEMS[1], 16);
      line1[0] = '>';  
    }
    else if(menuIdx == 2){
      strncpy_P(line0, MENU_ITEMS[1], 16);
      strncpy_P(line1, MENU_ITEMS[2], 16);
      line1[0] = '>';
    }
    else if(menuIdx == 3){
      strncpy_P(line0, MENU_ITEMS[2], 16);
      strncpy_P(line1, MENU_ITEMS[3], 16);
      line1[0] = '>';
    }
}

void manualMsg(){
    if(!switchDisplay){
      strncpy_P(line0, SET_MAN_LINE_00, 16);
      strncpy_P(line1, SET_MAN_LINE_01, 16);
      line0[14] = 0xDF;
      line1[14] = 0xDF;
      String value = "Tact: " + String(currentTemp,2);
      strncpy(line0,value.c_str(),value.length());
      value = "Ttar: " + String(targetTemp,2);
      strncpy(line1,value.c_str(),value.length());
    }
    
    else{
        strncpy_P(line0, SET_MAN_LINE_02, 16);
        strncpy_P(line1, SET_MAN_LINE_03, 16);
        String value = String(pwmValue,10);
        strncpy(line1,value.c_str(),value.length());
    }
}

void doFuzzy(void){
  if (millis() - lastFuzzy >= FUZZY_PERIOD_MS){
    lastFuzzy = millis();  //get ready for the next iteration
    bbqfan.handle();
  }
}

void doControll(void){

  double gap = targetTemp - currentTemp;
  if(gap <= -1.0){     
    pwmValue = 0;
  }
  else if(gap > 0 && gap<=10) {  //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
    myPID.Compute();
    pwmValue = map(outputVal, 0, 255, MIN_PWM, 255);
    if(pwmValue > MAX_PWM){
      pwmValue = MAX_PWM;
    }
  }
  else {//we're far from setpoint, use aggressive tuning parameters
    myPID.SetTunings(aggKp, aggKi, aggKd);
    myPID.Compute();
    pwmValue = map(outputVal, 0, 255, MIN_PWM, 255);
    if(pwmValue > MAX_PWM){
      pwmValue = MAX_PWM;
    }
  }
 }

void updateDisplay(void){
  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1); 
}
void loop(){
  // always read temperature
  currentTemp = readTemperature();
  uint8_t buttons = readButtons();
  switch (opState)
  {
    case INIT:
      lcd.begin(16, 2);
	    lcd.clear();
      opState = OFF;
      pwmValue = 0;
      outputVal = 0.0;
    break;
    case OFF:
      welcomeMsg();
      if (buttons & (1 << BUTTON_PUSH)){
        opState = SET_TEMP;
      }
      pwmValue = 0;
      break;
 
    case SET_TEMP:
      targetTempMsg();
      if (buttons  & (1<<LEFT_MOVE) && targetTemp < TARGET_TEMP_MAX){
         targetTemp += 1.0; 
      }
      else if (buttons & (1<<RIGHT_MOVE) && targetTemp > TARGET_TEMP_MIN){
         targetTemp -= 1.0;
      }
      if (buttons & (1 << BUTTON_PUSH)){
        opState = SET_TIMER;
      }
      break;
    
    case SET_TIMER:
      timerMsg();
      if (buttons  & (1<<LEFT_MOVE) && targetTimer < TIMER_MAX){
         targetTimer += 5; 
      }
      else if (buttons & (1<<RIGHT_MOVE) && targetTimer > TIMER_MIN){
         targetTimer -= 5;
      }
      if (buttons & (1 << BUTTON_PUSH)){
        opState = MENU;
      }
      break;

    case MENU:
      menuMsg();
      if (buttons  & (1<<LEFT_MOVE) && menuIdx < 3){
         menuIdx++; 
      }
      else if (buttons & (1<<RIGHT_MOVE) && menuIdx > 0){
         menuIdx--;
      }
      if (buttons & (1 << BUTTON_PUSH)){
        if(menuIdx == 0)
          opState = MANUAL_MODE;
        else if (menuIdx == 1)
          opState = PREP_PID;
        else if (menuIdx == 2)
          opState = PREP_FUZZY;
        else
          opState = INIT;
      }
      break;

    case MANUAL_MODE:
      manualMsg();
      if (buttons  & (1<<LEFT_MOVE) && pwmValue < MAX_PWM){
        pwmValue ++;
        switchDisplay = true;
        startMillisDisplay = currentMillis;
      }
      else if (buttons & (1<<RIGHT_MOVE) && pwmValue > MIN_PWM){
        pwmValue --;
        switchDisplay = true;
        startMillisDisplay = currentMillis;
      }
      currentMillisDisplay = millis();
      if(switchDisplay && currentMillisDisplay - startMillisDisplay >= periodDisplay){
        switchDisplay = false;  
      }

      if (buttons  & (1<<BUTTON_LONG_PUSH)){
        opState = INIT;
      }
      break;
    case PREP_FUZZY:
      bbqfan.init();
      opState = RUN_FUZZY;
    break;
    case RUN_FUZZY:
      manualMsg();
      doFuzzy();
      if (buttons  & (1<<LEFT_MOVE) || buttons & (1<<RIGHT_MOVE) ){
        switchDisplay = true;
        startMillisDisplay = currentMillis;
      }
      currentMillisDisplay = millis();
      if(switchDisplay && currentMillisDisplay - startMillisDisplay >= periodDisplay){
        switchDisplay = false;  
      }
      if (buttons  & (1<<BUTTON_LONG_PUSH)){ 
        opState = INIT;
      }
      break;
    
    case PREP_PID:
      myPID.SetMode(MANUAL);
      outputVal = 0.0;
      myPID.SetMode(AUTOMATIC);
      opState = RUN_PID;
      break;    
    
    case RUN_PID:
      manualMsg();
      doControll();
      if (buttons  & (1<<LEFT_MOVE) || buttons & (1<<RIGHT_MOVE) ){
        switchDisplay = true;
        startMillisDisplay = currentMillis;
      }
      currentMillisDisplay = millis();
      if(switchDisplay && currentMillisDisplay - startMillisDisplay >= periodDisplay){
        switchDisplay = false;  
      }
      if (buttons  & (1<<BUTTON_LONG_PUSH)){ 
        opState = INIT;
      }
      break;
    default:
      opState = INIT;
      break;
  }
  updateDisplay();
  analogWrite(pwmPin,pwmValue);
}


