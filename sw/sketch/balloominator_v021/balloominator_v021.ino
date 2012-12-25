#include <SPI.h>
#include <Wire.h>
#include <limits.h>
#include "ADXL345.h"

#define F_CPU 1000000L

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
}

unsigned long start;
void setup()
{
  unsigned char i;

  //unleash full speed
  CLKPR = (1 << CLKPCE); // enable a change to CLKPR
  CLKPR = 0; // set the CLKDIV to 0 - was 0011b = div by 8 

  pinMode(usb_present, INPUT);

  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  SPI.begin();
  Wire.begin();
  Serial.begin(57600);

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

#define T_LEN 1280
#define P_LEN 25600
#define FLASH_HALF 0.005
#define MAXB 255.0

void loop()
{
  unsigned char i;

  uint16_t t, t_tap;
  uint8_t p, flash;

  float pow;
  float ph;

  t = elapsedTime(start);
  t_tap = elapsedTime(last_tap);
  
  Serial.print(t, DEC);
  Serial.println();
  p = t % P_LEN;
  t %= (3 * T_LEN);
  
  //values[0] = 100.0 * sin(t / 900.0);
  //values[1] = 100.0 * cos(t / 900.0);
  //values[2] = 10.0 * exp(-t2);
  
  //pow = MAXB * (0.5 + 0.3 * sin((PI * p) / P_LEN));
  flash = MAXB/2 * exp(-FLASH_HALF * t_tap);
  if (flash > 1) {
    for (i=0; i<numLEDs; i++)
      values[i] = flash;
  }

  pow = MAXB;
  
  if(t < T_LEN)
  {
    ph = (PI * t) / (2 * T_LEN);
    values[0] = pow * sin(ph);
    values[1] = flash;
    values[2] = pow * cos(ph);
  }
  else if(t < 2*T_LEN)
  {
    ph = (PI * (t - T_LEN)) / (2 * T_LEN);
    values[0] = pow * cos(ph);
    values[1] = pow * sin(ph);
    values[2] = flash;
  }
  else 
  {
    ph = (PI * (t - 2 * T_LEN)) / (2 * T_LEN);
    values[0] = flash;
    values[1] = pow * cos(ph);
    values[2] = pow * sin(ph);
  }

  for (i=0; i<numLEDs; i++)
    analogWrite(ledPin[i], round(values[i]));
}
