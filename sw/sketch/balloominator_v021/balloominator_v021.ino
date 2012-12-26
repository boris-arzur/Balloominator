#include <SPI.h>
#include <ADXL345SPI.h>
#include <limits.h>

const unsigned char READ = 0x80;
const unsigned char usb_present = A5;

#define CS 8
Accelerometer accel(CS);

const unsigned char numLEDs = 3;

const unsigned char ledPin[] = {5, 6, 9}; // blue, green, red
const unsigned char test_led = 7;

unsigned long elapsedTime(unsigned long startTime)
{
  unsigned long stopTime = millis();
  
  if (stopTime >= startTime)
    return stopTime - startTime;
  else
    return (ULONG_MAX - (startTime - stopTime));
}


unsigned long last_tap;
void isr_tap()
{
  last_tap = millis();
}


void accelSetup()
{
  accel.powerOn(true); //highspeed !
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

  //force unleash full speed
  CLKPR = (1 << CLKPCE); // enable a change to CLKPR
  CLKPR = 0; // set the CLKDIV to 0 - was 0011b = div by 8 

  pinMode(usb_present, INPUT);
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
#define P_LEN 10000
#define FLASH_HALF 0.005
#define MAXB 255.0

#define XYZ_CHANGE 600.0
#define SCALE_TO_ILLUM 4.0
#define MIX_ 0.6

int x, y, z;
float px, py, pz;

inline float partial_norm2(int u, float pu)
{
  float tmp;
  tmp = u - pu;
  return tmp * tmp;
}

void loop()
{
  unsigned char i;

  uint16_t t, t_tap;
  uint8_t flash;

  float pow;
  float ph;
  
  float diff;

  t = elapsedTime(start);
  t_tap = elapsedTime(last_tap);
  
  //pow = t % P_LEN;
  t %= (3 * T_LEN);
  
  accel.readAccel(&x, &y, &z);
  diff = partial_norm2(x, px) +
    partial_norm2(y, py) +
    partial_norm2(z, pz);
    
  if (diff > XYZ_CHANGE)
    flash = diff / SCALE_TO_ILLUM;
  else
    flash = 0;
    
  px = MIX_ * px + (1.0 - MIX_) * x;
  py = MIX_ * py + (1.0 - MIX_) * y;
  pz = MIX_ * pz + (1.0 - MIX_) * z;
  
  for (i=0; i<numLEDs; i++)
    values[i] = flash;

  pow = 100.0;
  
  if(t < T_LEN)
  {
    ph = (PI * t) / (2 * T_LEN);
    values[0] += pow * sin(ph);
    values[2] += pow * cos(ph);
  }
  else if(t < 2*T_LEN)
  {
    ph = (PI * (t - T_LEN)) / (2 * T_LEN);
    values[0] += pow * cos(ph);
    values[1] += pow * sin(ph);
  }
  else 
  {
    ph = (PI * (t - 2 * T_LEN)) / (2 * T_LEN);
    values[1] += pow * cos(ph);
    values[2] += pow * sin(ph);
  }

  for (i=0; i<numLEDs; i++)
    analogWrite(ledPin[i], round(values[i]));
}
