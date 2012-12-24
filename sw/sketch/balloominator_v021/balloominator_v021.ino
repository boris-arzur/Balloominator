#include <SPI.h>
#include <Wire.h>
#include <limits.h>
#include "ADXL345.h"

#define UPDATE_INTERVAL 100
#define SCALE_VALUE 0.0003

const unsigned char cs = 8;
const unsigned char READ = 0x80;
const unsigned char usb_present = A5;


ADXL345 accel;

const unsigned char numLEDs = 3;

const unsigned char ledPin[] = {5, 6, 9}; // blue, green, red
const unsigned char test_led = 7;

unsigned long elapsedTime(unsigned long startTime)
{
  unsigned long stopTime = millis();
  
  if (stopTime >= startTime)
  {
    return stopTime - startTime;
  }
  else
  {
    return (ULONG_MAX - (startTime - stopTime));
  }
}


unsigned long last_tap;
void isr_tap()
{
  last_tap = millis();
  digitalWrite(test_led, HIGH);
  delay(10);
  digitalWrite(test_led, LOW);
}


void accelSetup()
{
  accel.powerOn();
  delay(10);
  accel.setRangeSetting(2);
  accel.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT, ADXL345_INT2_PIN);
  accel.setTapDetectionOnZ(true);
  accel.setTapThreshold(0x20);
  accel.setTapDuration(0xff);
  accel.setDoubleTapLatency(0x00);
  accel.setDoubleTapWindow(0x00);
  accel.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, true);
  accel.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, true);
  accel.getInterruptSource();

  attachInterrupt(0, isr_tap, RISING);
  attachInterrupt(1, isr_tap, RISING);
  
  digitalWrite(test_led, HIGH);
  delay(10);
  digitalWrite(test_led, LOW);
  delay(10);
  digitalWrite(test_led, HIGH);
  delay(10);
  digitalWrite(test_led, LOW);
}

unsigned long start;
void setup()
{
  unsigned char i;

  pinMode(usb_present, INPUT);

  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);

  pinMode(test_led, OUTPUT);

  for (i=0; i<numLEDs; i++)
  {
    pinMode(ledPin[i], OUTPUT);
    digitalWrite(ledPin[i], LOW);
  }
   
  accelSetup();
  
  start = millis();
}


double values[3] = {0.0, 0.0, 0.0};
#define T_LEN 2560
#define MAXB 255
void loop()
{
  unsigned char i;

  unsigned long t = elapsedTime(start);
  t %= 3*T_LEN;
  unsigned long t2 = elapsedTime(last_tap);
  
  //values[0] = 100.0 * sin(t / 900.0);
  //values[1] = 100.0 * cos(t / 900.0);
  //values[2] = 10.0 * exp(-t2);
  
  if(t < T_LEN)
  {
    //round the colour wheel
    //red up, green 0, blue down
    values[0] = MAXB*sin(PI*t/(2*T_LEN));
    values[1] = 0;
    values[2]= MAXB*cos(PI*t/(2*T_LEN));
  }
  else if(t < 2*T_LEN)
  {
    //round the colour wheel
    //red down, green up, blue 0
    t -= T_LEN;
    values[0] = MAXB*cos(PI*t/(2*T_LEN));
    values[1] = MAXB*sin(PI*t/(2*T_LEN));
    values[2] = 0;
  }
  else 
  {
    //round the colour wheel
    //red 0, green down, blue up
    t -= 2*T_LEN;
    values[0] = 0;
    values[1] = MAXB*cos(PI*t/(2*T_LEN));
    values[2] = MAXB*sin(PI*t/(2*T_LEN));
  }

  for (i=0; i<numLEDs; i++)
    analogWrite(ledPin[i], round(values[i]));
}

