#include <Arduino.h>
#include <avr/pgmspace.h>
#include <Wire.h>

#include <Adafruit_MAX31865.h>
#include <TimerFour.h>
#include <PID_v1.h>
#include <ss_oled.h>

#include "MyButton.h"
#include "RotaryEncoderSwitch.h"
#include "PlatinumSensor.h"
#include "bbqfan.h"


int pwmPin = 11;

// Use -1 for the Wire library default pins
// or specify the pin numbers to use with the Wire library or bit banging on any GPIO pins
// These are the pin numbers for the M5Stack Atom default I2C
#define SDA_PIN -1
#define SCL_PIN -1
// Set this to -1 to disable or the GPIO pin number connected to the reset
// line of your display if it requires an external reset
#define RESET_PIN -1
// let ss_oled figure out the display address
#define OLED_ADDR  0x3C
// don't rotate the display
#define FLIP180 0
// don't invert the display
#define INVERT 0
// Bit-Bang the I2C bus
#define USE_HW_I2C 1

#define MY_OLED OLED_128x64
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
SSOLED ssoled;

Adafruit_MAX31865 max = Adafruit_MAX31865(7,8,9,10);
RotaryEnoderSwitch rot = RotaryEnoderSwitch(6,5,EncoderType::twoStep);
MyButton mybutton = MyButton(4,true,false);

enum operatingState { INIT = 0, OFF,SET_TEMP, SET_TIMER, MENU, MANUAL_MODE, PREP_FUZZY, RUN_FUZZY, PREP_PID, RUN_PID};
operatingState opState = OFF;

#define RIGHT_MOVE 0
#define LEFT_MOVE 1
#define BUTTON_PUSH 2
#define BUTTON_LONG_PUSH 3


uint16_t startMillis;  //some global variables available anywhere in the program
uint16_t currentMillis;
uint16_t period = 1000;  //the value is a number of milliseconds

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

const char s_00[] PROGMEM = "BBQ is Off    \0";
const char s_01[] PROGMEM = "Press Button  \0";
const char s_02[] PROGMEM = "Set Temp in °C\0";
const char s_03[] PROGMEM = "Set Timer min \0";
const char s_04[] PROGMEM = " Manual Mode  \0";
const char s_05[] PROGMEM = " PID Control  \0";
const char s_06[] PROGMEM = " Fuzzy        \0";     
const char s_07[] PROGMEM = " Reset        \0";
const char s_08[] PROGMEM = "PWM:            \0";
const char s_09[] PROGMEM = "Tact:           \0";
const char s_10[] PROGMEM = "Ttar:           \0";     
const char s_11[] PROGMEM = "Timer           \0";
const char s_12[] PROGMEM = "                \0";


const char *const string_table[] PROGMEM = {s_00,s_01,s_02,s_03,s_04,s_05,s_06,s_07,s_08,s_09,s_10,s_11,s_12};

char buffer[30];
char* fillBuffer(int i){
  memset(buffer, 0, sizeof(buffer));
  strcpy_P(buffer, (char *)pgm_read_word(&(string_table[i])));
  return buffer;
}
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
  max.begin(MAX31865_4WIRE);
  Timer4.initialize(1000);
  Timer4.attachInterrupt(timer4);
  opState = INIT;
  oledInit(&ssoled, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L); // use standard I2C bus at 400Khz
 }

float readTemperature(void){
  float rtd = (float)max.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  // Check and print any faults
  float temp = PlatinumSensor::tempFromPtResistance(4300.00*ratio,1000.00);
  uint8_t fault = max.readFault();
  if (fault) {
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
  
  oledWriteString(&ssoled, 0,0,0,fillBuffer(0), FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0,0,1,fillBuffer(1), FONT_NORMAL, 0, 1);
}

void targetTempMsg(void){
  oledWriteString(&ssoled, 0,0,0,fillBuffer(2), FONT_NORMAL, 0, 1);
  dtostrf(targetTemp, 3, 0, buffer);
  oledWriteString(&ssoled, 0,0,3, buffer , FONT_NORMAL, 0, 1);
}

void timerMsg(void){
  oledWriteString(&ssoled, 0,0,0,fillBuffer(3), FONT_NORMAL, 0, 1);
  // seems to be stupid but itoa does not right padding and sprintf eats my flash
  dtostrf(targetTimer, 3, 0, buffer);
  oledWriteString(&ssoled, 0,0,3, buffer , FONT_NORMAL, 0, 1);
}

void menuMsg(){ //TODO rework scrolling too many ifs
  for(uint8_t i =0; i < 4; i++){
    fillBuffer(4+i);
    if(i == menuIdx){
      buffer[0] ='*';
    }
    oledWriteString(&ssoled, 0,0,i,buffer, FONT_NORMAL, 0, 1);
  }
}

void manualMsg(){
  fillBuffer(8);
  dtostrf(pwmValue, 6, 0, &buffer[5]);
  oledWriteString(&ssoled, 0,0,0,buffer, FONT_NORMAL, 0, 1);
  fillBuffer(9);
  dtostrf(currentTemp, 6, 1, &buffer[7]);
  oledWriteString(&ssoled, 0,0,2,buffer, FONT_NORMAL, 0, 1);
  fillBuffer(10);
  dtostrf(targetTemp, 6, 1, &buffer[7]);
  oledWriteString(&ssoled, 0,0,4,buffer, FONT_NORMAL, 0, 1);
  fillBuffer(11);
  dtostrf(targetTimer, 6, 0, &buffer[5]);
  oledWriteString(&ssoled, 0,0,6,buffer, FONT_NORMAL, 0, 1);
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

void loop(){
  // always read temperature
  currentTemp = readTemperature();
  uint8_t buttons = readButtons();
  switch (opState)
  {
    case INIT:
      opState = OFF;
      pwmValue = 0;
      outputVal = 0.0;
      oledFill(&ssoled, 0, 1);
    break;
    case OFF:
      welcomeMsg();
      if (buttons & (1 << BUTTON_PUSH)){
        opState = SET_TEMP;
        oledFill(&ssoled, 0, 1);
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
        oledFill(&ssoled, 0, 1);
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
        oledFill(&ssoled, 0, 1);  
      }
      break;

    case MANUAL_MODE:
      manualMsg();
      if (buttons  & (1<<LEFT_MOVE) && pwmValue < MAX_PWM){
        pwmValue ++;
      }
      else if (buttons & (1<<RIGHT_MOVE) && pwmValue > MIN_PWM){
        pwmValue --;
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
      if (buttons  & (1<<BUTTON_LONG_PUSH)){ 
        opState = INIT;
      }
      break;
    default:
      opState = INIT;
      break;
  }
  analogWrite(pwmPin,pwmValue);
}


