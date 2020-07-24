#include <Arduino.h>
#include <avr/pgmspace.h>

#include <Adafruit_MAX31865.h>
#include <TimerFour.h>
#include <PID_v1.h>

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

#include "MyButton.h"
#include "RotaryEncoderSwitch.h"
#include "PlatinumSensor.h"
#include "bbqfan.h"

int pwmPin = 11;

#define OLED_ADDR 0x3C

//SSOLED ssoled;
SSD1306AsciiWire oled;

Adafruit_MAX31865 max = Adafruit_MAX31865(7, 8, 9, 10);
RotaryEnoderSwitch rot = RotaryEnoderSwitch(6, 5, EncoderType::twoStep);
MyButton mybutton = MyButton(4, true, false);

enum operatingState
{
  INIT = 0,
  OFF,
  SET_TEMP,
  SET_TIMER,
  MENU,
  MANUAL_MODE,
  PREP_FUZZY,
  RUN_FUZZY,
  PREP_PID,
  RUN_PID,
  AUTO_TIMER,
  FINISH
};
operatingState opState = OFF;
operatingState history = OFF;

#define RIGHT_MOVE 0
#define LEFT_MOVE 1
#define BUTTON_PUSH 2
#define BUTTON_LONG_PUSH 3

uint16_t startMillis; //some global variables available anywhere in the program
uint16_t currentMillis;
uint16_t period = 1000; //the value is a number of milliseconds

double currentTemp = 0.0;
double targetTemp = 60.0;
#define TARGET_TEMP_MAX 180.0
#define TARGET_TEMP_MIN 20.0
#define TIMER_MAX 840
#define TIMER_MIN 5
uint32_t targetTimer = TIMER_MIN;
uint32_t countDown = 0;
uint32_t lastTime = 0;

uint8_t pwmValue = 0;
#define MAX_PWM 255
#define MIN_PWM 30

double outputVal;
double aggKp = 8, aggKi = 0.2, aggKd = 1;
double consKp = 2, consKi = 0.05, consKd = 0.25;

PID myPID(&currentTemp, &outputVal, &targetTemp, consKp, consKi, consKd, DIRECT);
BBQFan bbqfan(currentTemp, targetTemp, pwmValue);
unsigned long lastFuzzy = 0;
#define FUZZY_PERIOD_MS 5000

const char s_00[] = "BBQ is Off    \0";
const char s_01[] = "Press Button  \0";
const char s_02[] = "Set Temp in  C\0";
const char s_03[] = "Set Timer min \0";
const char s_04[] = " Manual Mode  \0";
const char s_05[] = " PID Control  \0";
const char s_06[] = " Fuzzy        \0";
const char s_07[] = " Reset        \0";
const char s_08[] = "PWM:           \0";
const char s_09[] = "Tact           \0";
const char s_10[] = "Ttar           \0";
const char s_11[] = "Timer          \0";
const char s_12[] = "               \0";
const char s_13[] = "**** Ready ****\0";
const char s_14[] = "** Enjoy it! **\0";
const char *const string_table[] = {s_00, s_01, s_02, s_03, s_04, s_05, s_06, s_07, s_08, s_09, s_10, s_11, s_12, s_13, s_14};

uint8_t menuIdx = 0;

char buffer[30];
char *fillBuffer(int i)
{
  memset(buffer, 0, sizeof(buffer));
  strcpy(buffer, string_table[i]);
  return buffer;
}

void timer4(void)
{
  rot.tickDebounceDecode();
  mybutton.debounce();
}

void setup()
{
  // PWM on Pin 3
  //http://www.scynd.de/tutorials/arduino-tutorials/3-luefter-steuern/3-1-pwm-ohne-pfeifen.html
  //TCCR1B = TCCR1B & 0b11111000 | 0x01;
  pinMode(pwmPin, OUTPUT);
  max.begin(MAX31865_4WIRE);
  Timer4.initialize(1000);
  Timer4.attachInterrupt(timer4);
  opState = INIT;
  Wire.begin();
  Wire.setClock(400000L);
  oled.begin(&SH1106_128x64, OLED_ADDR);
  oled.setFont(Adafruit5x7);
  oled.clear();
}

float readTemperature(void)
{
  float rtd = (float)max.readRTD();
  float ratio = rtd;
  ratio /= 32768;
  // Check and print any faults
  float temp = PlatinumSensor::tempFromPtResistance(4300.00 * ratio, 1000.00);
  uint8_t fault = max.readFault();
  if (fault)
  {
    max.clearFault();
  }
  return temp;
}

uint8_t readButtons()
{
  uint8_t buttons = 0;
  int8_t idx = rot.readEncoder();
  if (idx > 0)
    buttons |= (1 << LEFT_MOVE);
  else if (idx < 0)
    buttons |= (1 << RIGHT_MOVE);

  currentMillis = millis();
  if (mybutton.buttonPressed() && (currentMillis - startMillis >= period))
  {
    startMillis = currentMillis;
    if (mybutton.longPressed())
      buttons |= (1 << BUTTON_LONG_PUSH);
    else
      buttons |= (1 << BUTTON_PUSH);
  }
  return buttons;
}

void welcomeMsg(void)
{
  oled.setCursor(0, 0);
  oled.println(fillBuffer(0));
  oled.println(fillBuffer(1));
}

void targetTempMsg(void)
{
  oled.setCursor(0, 0);
  oled.println(fillBuffer(2));
  oled.setCursor(0, 3);
  oled.print(targetTemp, 0);
}

void timerMsg(void)
{
  oled.setCursor(0, 0);
  oled.println(fillBuffer(3));
  uint8_t hours = targetTimer / 60;
  uint8_t minutes = targetTimer % 60;
  countDown = targetTimer * 60;
  dtostrf(hours, 2, 0, &buffer[0]);
  buffer[2] = ':';
  dtostrf(minutes, 2, 0, &buffer[3]);
  if (minutes < 10)
    buffer[3] = '0';
  oled.setCursor(0, 3);
  oled.println(buffer);
}

void menuMsg()
{ //TODO rework scrolling too many ifs
  oled.setCursor(0, 0);
  for (uint8_t i = 0; i < 4; i++)
  {
    fillBuffer(4 + i);
    if (i == menuIdx)
    {
      buffer[0] = '*';
    }
    oled.println(buffer);
  }
}

void manualMsg()
{
  oled.setCursor(0, 0);

  fillBuffer(8);
  dtostrf(pwmValue, 3, 0, &buffer[7]);
  oled.println(buffer);
  oled.println();

  fillBuffer(9);
  dtostrf(currentTemp, 6, 1, &buffer[7]);
  oled.println(buffer);
  oled.println();

  fillBuffer(10);
  dtostrf(targetTemp, 6, 1, &buffer[7]);
  oled.println(buffer);
  oled.println();

  fillBuffer(11);
  uint8_t hours = (countDown / 3600);
  uint8_t minutes = (countDown - (3600 * hours)) / 60;
  uint8_t seconds = (countDown - (3600 * hours) - (minutes * 60));
  dtostrf(hours, 2, 0, &buffer[6 + 0]);
  buffer[6 + 2] = ':';
  dtostrf(minutes, 2, 0, &buffer[6 + 3]);
  if (minutes < 10)
    buffer[6 + 3] = '0';
  buffer[6 + 5] = ':';
  dtostrf(seconds, 2, 0, &buffer[6 + 6]);
  if (seconds < 10)
    buffer[6 + 6] = '0';
  oled.println(buffer);
}

void finishMsg(void)
{

  oled.setInvertMode(true);
  oled.setCursor(0, 0);
  oled.println(fillBuffer(13));
  oled.println(fillBuffer(14));
}

void doControll(void)
{
  if (opState == RUN_PID)
  {
    double gap = targetTemp - currentTemp;
    if (gap <= -1.0)
    {
      pwmValue = 0;
    }
    else if (gap > 0 && gap <= 10)
    { //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
      myPID.Compute();
      pwmValue = map(outputVal, 0, 255, MIN_PWM, 255);
      if (pwmValue > MAX_PWM)
      {
        pwmValue = MAX_PWM;
      }
    }
    else
    { //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
      myPID.Compute();
      pwmValue = map(outputVal, 0, 255, MIN_PWM, 255);
      if (pwmValue > MAX_PWM)
      {
        pwmValue = MAX_PWM;
      }
    }
  }
  else if (opState == RUN_FUZZY)
  {
    if (millis() - lastFuzzy >= FUZZY_PERIOD_MS)
    {
      lastFuzzy = millis(); //get ready for the next iteration
      bbqfan.handle();
    }
  }
}

void loop()
{
  // always read temperature
  currentTemp = readTemperature();
  uint8_t buttons = readButtons();
  if (buttons & (1 << BUTTON_LONG_PUSH))
  {
    opState = INIT;
  }
  switch (opState)
  {
  case INIT:
    opState = OFF;
    pwmValue = 0;
    outputVal = 0.0;
    oled.setInvertMode(false);
    oled.clear();
    break;
  case OFF:
    welcomeMsg();
    if (buttons & (1 << BUTTON_PUSH))
    {
      opState = SET_TEMP;
      oled.clear();
    }
    pwmValue = 0;
    break;

  case SET_TEMP:
    targetTempMsg();
    if (buttons & (1 << LEFT_MOVE) && targetTemp < TARGET_TEMP_MAX)
    {
      targetTemp += 1.0;
    }
    else if (buttons & (1 << RIGHT_MOVE) && targetTemp > TARGET_TEMP_MIN)
    {
      targetTemp -= 1.0;
    }
    if (buttons & (1 << BUTTON_PUSH))
    {
      opState = SET_TIMER;
      oled.clear();
    }
    break;

  case SET_TIMER:
    timerMsg();
    if (buttons & (1 << LEFT_MOVE) && targetTimer < TIMER_MAX)
    {
      targetTimer += 5;
    }
    else if (buttons & (1 << RIGHT_MOVE) && targetTimer > TIMER_MIN)
    {
      targetTimer -= 5;
    }
    if (buttons & (1 << BUTTON_PUSH))
    {
      opState = MENU;
    }
    break;
    countDown = targetTimer * 60000;

  case MENU:
    menuMsg();
    if (buttons & (1 << LEFT_MOVE) && menuIdx < 3)
    {
      menuIdx++;
    }
    else if (buttons & (1 << RIGHT_MOVE) && menuIdx > 0)
    {
      menuIdx--;
    }
    if (buttons & (1 << BUTTON_PUSH))
    {
      if (menuIdx == 0)
        opState = MANUAL_MODE;
      else if (menuIdx == 1)
        opState = PREP_PID;
      else if (menuIdx == 2)
        opState = PREP_FUZZY;
      else
        opState = INIT;
      oled.clear();
    }
    break;

  case MANUAL_MODE:
    manualMsg();
    if (buttons & (1 << LEFT_MOVE) && pwmValue < MAX_PWM)
    {
      pwmValue++;
    }
    else if (buttons & (1 << RIGHT_MOVE) && pwmValue > 0)
    {
      pwmValue--;
    }
    history = opState;
    opState = AUTO_TIMER;
    break;
  case PREP_FUZZY:
    bbqfan.init();
    opState = RUN_FUZZY;
    break;
  case RUN_FUZZY:
    manualMsg();
    doControll();
    history = opState;
    opState = AUTO_TIMER;
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
    history = opState;
    opState = AUTO_TIMER;
    break;
  case AUTO_TIMER:
    opState = history;
    if (millis() - lastTime >= 1000)
    {
      lastTime = millis();
      countDown -= 1;
    }
    if (countDown == 0)
    {
      oled.clear();
      opState = FINISH;
    }
    break;
  case FINISH:
    finishMsg();
    break;
  default:
    opState = INIT;
    break;
  }
  analogWrite(pwmPin, pwmValue);
}
