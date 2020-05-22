#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Adafruit_MAX31865.h>
#include <TimerFour.h>

#include "PlatinumSensor.h"

int pwmPin = 3;
int8_t loopi=0;

LiquidCrystal lcd(7,8,9,10,11,12);

Adafruit_MAX31865 max = Adafruit_MAX31865(2,4,5,6);

const int led = LED_BUILTIN;  // the pin with a LED
// The interrupt will blink the LED, and keep
// track of how many times it has blinked.
int ledState = LOW;
volatile unsigned long blinkCount = 0; // use volatile for shared variables

void blinkLED(void)
{
  if (ledState == LOW) {
    ledState = HIGH;
    blinkCount = blinkCount + 1;  // increase when LED turns on
  } else {
    ledState = LOW;
  }
  digitalWrite(led, ledState);
}


void setup() {
  // PWM on Pin 3
  //http://www.scynd.de/tutorials/arduino-tutorials/3-luefter-steuern/3-1-pwm-ohne-pfeifen.html
  //TCCR1B = TCCR1B & 0b11111000 | 0x01;
  pinMode(pwmPin, OUTPUT);
  // put your setup code here, to run once:
  // set up the LCD's number of columns and rows:
	lcd.begin(16, 2);
	// Clears the LCD screen
	lcd.clear();
  Serial.begin(115200);
  max.begin(MAX31865_4WIRE);

  pinMode(led, OUTPUT);
  Timer4.initialize(150000);
  Timer4.attachInterrupt(blinkLED); // blinkLED to run every 0.15 seconds
}

void loop() {
  float rtd = (float)max.readRTD();
  Serial.print("RTD value: "); Serial.println(rtd);
  float ratio = rtd;
  ratio /= 32768;
  Serial.print("Ratio = "); Serial.println(ratio,8);
  Serial.print("Resistance = "); Serial.println(4300.00*ratio,8);
  Serial.print("Temp °C = "); Serial.println(PlatinumSensor::tempFromPtResistance(4300.00*ratio,1000.00));
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
  delay(1000);
  lcd.clear();
  
  /*
  int start=0;
  while(true)
  {
    analogWrite(pwmPin,start);
    delay(300);
    Serial.println(start%255); 
    start+=1;
  }*/

}


