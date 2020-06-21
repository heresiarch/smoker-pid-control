#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <Adafruit_MAX31865.h>
#include <TimerFour.h>


#include "MyButton.h"
#include "RotaryEncoderSwitch.h"
#include "PlatinumSensor.h"


int pwmPin = 3;
LiquidCrystal lcd(7,8,9,10,11,12);
char line0[21]; 
char line1[21];

Adafruit_MAX31865 max = Adafruit_MAX31865(2,4,5,6);
RotaryEnoderSwitch rot = RotaryEnoderSwitch(A0,A1,EncoderType::twoStep);
MyButton mybutton = MyButton(A2,true,false);

enum operatingState { OFF = 0, SET_T, RUN, FULL};
operatingState opState = OFF;

#define RIGHT_MOVE 0
#define LEFT_MOVE 1
#define BUTTON_PUSH 2
#define BUTTON_LONG_PUSH 3

float targetTemp = 90.0;
#define TARGET_TEMP_MAX 150.0
#define TARGET_TEMP_MIN 90
                                            //0123456789ABCDEF     
static const char OFF_LINE_00[] PROGMEM =   "BBQ is Off      ";
static const char OFF_LINE_01[] PROGMEM =   "Press Button    ";
static const char SET_T_LINE_00[] PROGMEM = "Set Temperature ";
static const char SET_T_LINE_01[] PROGMEM = "              CC";

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
  
  if(mybutton.buttonPressed()){
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

void targetTempMsg(){
  strncpy_P(line0, SET_T_LINE_00, 16);
  strncpy_P(line1, SET_T_LINE_01, 16);
  line1[14] = 0xDF;
  line1[15] = 'C';
  String value = String(targetTemp,0);
  strncpy(line1,value.c_str(),value.length());
  Serial.print(line1);
}



void updateDisplay(void){
  lcd.setCursor(0, 0);
  lcd.print(line0);
  lcd.setCursor(0, 1);
  lcd.print(line1); 
}

void loop(){
  // always read temperature
  float temp = readTemperature();
  uint8_t buttons = readButtons();
  switch (opState)
  {
    case OFF:
      welcomeMsg();
      if (buttons & (1 << BUTTON_PUSH)){
        opState = SET_T;
      }
      break;
 
    case SET_T:
      targetTempMsg();
      if (buttons  & (1<<LEFT_MOVE) && targetTemp < TARGET_TEMP_MAX){
         targetTemp += 1.0; 
      }
      else if (buttons & (1<<RIGHT_MOVE) && targetTemp > TARGET_TEMP_MIN){
         targetTemp -= 1.0;
      }
      if (buttons & (1 << BUTTON_PUSH)){
        opState = OFF;
      }
      break;
    
    case RUN:
      //runMotor(1);
      //if (sigControl.isActive())
      //  state = CLOSE2OPEN;  // reverse the motion
      //else if (sigLimitClose.isActive())
      //  state = CLOSE;       // reached the end of motion
      break;
    case FULL:

      break;    

    default:
      opState = OFF;
      break;
  }
  updateDisplay();
    
}


/*
void loop() {
  float rtd = (float)max.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  // Check and print any faults
  lcd.print("R="); lcd.print(4300.00*ratio);
  lcd.setCursor(0,1); 
  lcd.print("T="); lcd.print(PlatinumSensor::tempFromPtResistance(4300.00*ratio,1000.00));
  uint8_t fault = max.readFault();
  if (fault) {
    Serial.print("Fault 0x"); Serial.println(fault, HEX);
    if (fault & MAX31865_FAULT_HIGHTHRESH) {
      Serial.println("RTD High Threshold"); 
    }
    if (fault & MAX31865_FAULT_LOWTHRESH) {
      Serial.println("RTD Low Threshold"); 
    }
    if (fault & MAX31865_FAULT_REFINLOW) {
      Serial.println("REFIN- > 0.85 x Bias"); 
    }
    if (fault & MAX31865_FAULT_REFINHIGH) {
      Serial.println("REFIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_RTDINLOW) {
      Serial.println("RTDIN- < 0.85 x Bias - FORCE- open"); 
    }
    if (fault & MAX31865_FAULT_OVUV) {
      Serial.println("Under/Over voltage"); 
    }
    max.clearFault();
  }
  int8_t delta = rot.readEncoder();
  if(delta < 0)
    Serial.println("Right");
  else if(delta > 0)
      Serial.println("Left");
  
  if(mybutton.buttonPressed()){
    Serial.println("Button");
  }
  if(mybutton.longPressed()){
      Serial.println("Long");  
  }
  delay(100);
  while(true)
  {
    analogWrite(pwmPin,start);
    delay(300);
    Serial.println(start%255); 
    start+=1;
  }

}*/


