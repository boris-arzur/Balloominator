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

unsigned long last_tap;
void isr_tap()
{
  last_tap = millis();
}

void accelSetup()
{
  accel.powerOn(true); //highspeed !
  //accel.setRangeSetting(2);
  //accel.setInterruptMapping(ADXL345_INT_SINGLE_TAP_BIT, ADXL345_INT2_PIN);
  //accel.setTapDetectionOnZ(true);
  //accel.setTapThreshold(0x20);
  //accel.setTapDuration(0xff);
  //accel.setDoubleTapLatency(0x00);
  //accel.setDoubleTapWindow(0x00);
  //accel.setInterrupt(ADXL345_INT_SINGLE_TAP_BIT, true);
  //accel.setInterrupt(ADXL345_INT_DOUBLE_TAP_BIT, true);
  //accel.getInterruptSource();

  //attachInterrupt(0, isr_tap, RISING);
  //attachInterrupt(1, isr_tap, RISING);
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
}


double values[3] = {0.0, 0.0, 0.0};

#define T_LEN 1280
#define P_LEN 10000
#define FLASH_HALF 0.005
#define MAXB 255.0

#define XYZ_CHANGE 600.0
#define SCALE_TO_ILLUM 4.0

int x, y, z;
int px, py, pz;

inline int partial_norm2(int u, int pu)
{
  int tmp;
  tmp = u - pu;
  return tmp * tmp;
}

float x2, x3;
inline float MAXB_sin_0pi2(float x)
{
  x2 = x * x;
  x3 = x2 * x;
  return 256.0 * x - 43.0 * x3;
  //return MAXB * x - MAXB * x3 / 6.0 + MAXB * x2 * x3 / 120.0;
}

inline float MAXB_cos_0pi2(float x)
{
  return 255.0 - 126.0 * x2 + 9.0 * x2 * x2;
  //return MAXB - MAXB * x2 / 2.0 + MAXB * x4 / 24.0;
}

void loop()
{
  unsigned char i;

  uint16_t t;

  float ph;
  
  int diff;
  accel.readAccel(&x, &y, &z);
  diff = partial_norm2(x, px) +
    partial_norm2(y, py) +
    partial_norm2(z, pz);
  
  px = (px >> 1) + (x >> 1);
  py = (py >> 1) + (y >> 1);
  pz = (pz >> 1) + (z >> 1); 
  
  /* flash *//*
#define SET +=
#define ZERO(arg) {}
  uint8_t flash;
  if (diff > XYZ_CHANGE)
    flash = diff >> 2;
  else
    flash = 0;
 
  for (i=0; i<numLEDs; i++)
    values[i] = flash;

  /**/
  
  t = millis();

  /* fast forward *///*
#define SET =
#define ZERO(arg) arg = 0
  uint16_t fast_fwd;
  
  if (diff > XYZ_CHANGE)
    fast_fwd += diff >> 2;
  t += fast_fwd;
  if (fast_fwd)
    fast_fwd--;
  /**/
  
  t %= (3 * T_LEN);
  if(t < T_LEN)
  {
    ph = (PI * t) / (2 * T_LEN);
    values[0] SET MAXB_sin_0pi2(ph);
    ZERO(values[1]);
    values[2] SET MAXB_cos_0pi2(ph);
  }
  else if(t < 2*T_LEN)
  {
    ph = (PI * (t - T_LEN)) / (2 * T_LEN);
    values[1] SET MAXB_sin_0pi2(ph); // always calc sin before cos
    values[0] SET MAXB_cos_0pi2(ph);
    ZERO(values[2]);
  }
  else 
  {
    ph = (PI * (t - 2 * T_LEN)) / (2 * T_LEN);
    values[2] SET MAXB_sin_0pi2(ph); // always calc sin before cos
    values[1] SET MAXB_cos_0pi2(ph);
    ZERO(values[0]);
  }

  //Serial.println(values[0], DEC);
  //Serial.println(ph, DEC);

  for (i=0; i<numLEDs; i++)
    analogWrite(ledPin[i], round(values[i]));
}
