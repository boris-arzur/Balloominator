#include <chibi.h>
#include <SPI.h>
#include <Wire.h>
#include <limits.h>
#include "ADXL345.h"

#define UPDATE_INTERVAL 100
#define SCALE_VALUE 0.0003

const unsigned char cs = 8;
const unsigned char READ = 0x80;
const unsigned char usb_present = A5;
unsigned char leds_off = false;

ADXL345 accel;

unsigned char numPatterns = 17;
unsigned char numLEDs = 3;
unsigned char fade = 0;
double scaleFactor = 0.0;
static unsigned char currPattern = 0;
unsigned long start;
unsigned char changeFlag = 0;

unsigned char ledPattern[][3] =
{
  {0xff, 0x00, 0x00},
  {0x00, 0xff, 0x00},
  {0x00, 0x00, 0xff},
  {0xff, 0xff, 0x00},
  {0xff, 0x00, 0xff},
  {0x00, 0xff, 0xff},
  {0xff, 0xff, 0xff},
  {0xff, 0x1f, 0x00},
  {0x00, 0x1f, 0xff},
  {0xff, 0x1f, 0xff},
  {0x1f, 0xff, 0x00},
  {0x1f, 0x00, 0xff},
  {0x1f, 0xff, 0xff},
  {0xff, 0x00, 0x1f},
  {0x00, 0xff, 0x1f},
  {0xff, 0xff, 0x1f},
  {0xff, 0xff, 0xff}  
};

const unsigned char ledPin[] = {5, 6, 9}; // blue, green, red

/**************************************************************************/
/*!
*/
/**************************************************************************/
void setup()
{
  unsigned char i;
  
  chibiCmdInit(57600);
  SPI.begin();
  
  pinMode(usb_present, INPUT);
  
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  
  for (i=0; i<numLEDs; i++)
  {
    pinMode(ledPin[i], OUTPUT);
    digitalWrite(ledPin[i], LOW);
  }
  
  chibiCmdAdd("rspi", cmd_read_spi);
  chibiCmdAdd("wspi", cmd_write_spi);
  chibiCmdAdd("racc", cmd_read_accel);
  chibiCmdAdd("rd", cmd_read_regs);
  chibiCmdAdd("wr", cmd_write_regs);
  chibiCmdAdd("print", cmd_print_all);
  
  // add isr
  attachInterrupt(0, isr_tap0, RISING);
  attachInterrupt(1, isr_tap1, RISING);
  
  accel.powerOn();
  tap_setup();
  
  start = millis();
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void loop()
{
  unsigned int i, val;
  chibiCmdPoll();
  
  val = analogRead(usb_present);
  
  // check to make sure VBUS is not present
  if (val > 1000)
  {
//    leds_off = true;
//    for (i=0; i<numLEDs; i++)
//    {
//      analogWrite(ledPin[i], 0);
//    }
  }
  else
  {
    leds_off = false;
  }
  
  if (elapsedTime(start) > UPDATE_INTERVAL)
  {   
    // fade the LEDs on and off
    if (fade == 0)
    {
      if (scaleFactor < 0.01)
      {
        // start fading up
        fade = 1;
      }
      else
      {
        if (scaleFactor > 0.6) scaleFactor -= SCALE_VALUE; 
        else if (scaleFactor > 0.3) scaleFactor -= SCALE_VALUE/2;
        else scaleFactor -= SCALE_VALUE/10;
//        else if (scaleFactor > 0.1) scaleFactor -= SCALE_VALUE/5;
      }
    }
    else
    {
      if (scaleFactor > 1.0)
      {
        fade = 0;
      }
      else
      {
        if (scaleFactor > 0.6) scaleFactor += SCALE_VALUE; 
        else if (scaleFactor > 0.3) scaleFactor += SCALE_VALUE/2;
        else scaleFactor += SCALE_VALUE/10;
//        else if (scaleFactor > 0.1) scaleFactor += SCALE_VALUE/5;
      }
    }
  
  // if a tap occurred, then set scaleFactor to 1 and increment currPattern
  if (changeFlag == true)
  {
    delay(100);
    changeFlag = false;
        
    // increment pattern circularly
    currPattern = ++currPattern % numPatterns;
    
    // full scale on change
    scaleFactor = 1.0;
  }
  
    // update the LEDs
//    Serial.println(scaleFactor, 4);
    if (leds_off == false)
    {
      for (i=0; i<numLEDs; i++)
      {
        unsigned char brightness = round(ledPattern[currPattern][i] * scaleFactor);
        
        //analogWrite(ledPin[i], random(0, 255));
        analogWrite(ledPin[i], brightness);
      }
    }
  }
}

/**************************************************************************/
/*!
  Setup the tap parameters
*/
/**************************************************************************/
void tap_setup()
{
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
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void isr_tap0()
{
  unsigned char i;
  
  unsigned char isr = accel.getInterruptSource();
  if (isr & (1<<ADXL345_INT_SINGLE_TAP_BIT))
  {
    Serial.println("Single Tap 0");    
    changeFlag = 1;
  }
  
  if (isr & (1<<ADXL345_INT_DOUBLE_TAP_BIT))
  {
     Serial.println("Double Tap 0");       
  } 
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void isr_tap1()
{
  byte isr = accel.getInterruptSource();
  
  Serial.println(isr, HEX);
  
  if (isr & (1<<ADXL345_INT_SINGLE_TAP_BIT))
  {
    Serial.println("Single Tap 1");
  }
  
  if (isr & (1<<ADXL345_INT_DOUBLE_TAP_BIT))
  {
     Serial.println("Double Tap 1");       
  } 
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
byte spiRead(byte addr)
{
  byte val;
  
  digitalWrite(cs, LOW);
  val = addr | READ;
  SPI.transfer(val);
  val = SPI.transfer(0);
  digitalWrite(cs, HIGH);
  return val;
}


/**************************************************************************/
/*!
*/
/**************************************************************************/
void spiWrite(byte addr, byte val)
{ 
  digitalWrite(cs, LOW);
  SPI.transfer(addr);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
}

/**************************************************************************/
/*!
    Read regs
*/
/**************************************************************************/
void cmd_read_spi(int arg_cnt, char **args)
{
  byte addr, val;
  addr = chibiCmdStr2Num(args[1], 16);
  val = spiRead(addr);
  Serial.print("Addr "); Serial.print(addr, HEX); Serial.print(" = "); Serial.println(val, HEX);
}

/**************************************************************************/
/*!
*/
/**************************************************************************/
void cmd_write_spi(int arg_cnt, char **args)
{
  byte addr, val;
  addr = chibiCmdStr2Num(args[1], 16);
  val = chibiCmdStr2Num(args[2], 16);
  spiWrite(addr, val);
  val = spiRead(addr);
  Serial.print("Addr "); Serial.print(addr, HEX); Serial.print(" = "); Serial.println(val, HEX);
}

/**************************************************************************/
/*!
    Read regs
*/
/**************************************************************************/
void cmd_read_accel(int arg_cnt, char **args)
{
  int x, y, z;
  accel.readAccel(&x, &y, &z);
  Serial.print(x, DEC);
  Serial.print(" ");
  Serial.print(y, DEC);
  Serial.print(" ");
  Serial.print(z, DEC);
  Serial.println("");
}

/**************************************************************************/
/*!
    Read regs
*/
/**************************************************************************/
void cmd_read_regs(int arg_cnt, char **args)
{
  unsigned char addr, val;
  
  addr = chibiCmdStr2Num(args[1], 16);

  val = *(volatile unsigned char *)addr;
  Serial.print("Addr "); Serial.print(addr, HEX); Serial.print(" = "); Serial.println(val, HEX);
}

/**************************************************************************/
/*!
    Write regs
*/
/**************************************************************************/
void cmd_write_regs(int arg_cnt, char **args)
{
  unsigned char addr, val;
  
  addr = chibiCmdStr2Num(args[1], 16);
  val = chibiCmdStr2Num(args[2], 16);
  
  *(volatile unsigned char *)addr = val;
  
  val = *(volatile unsigned char *)addr;
  Serial.print("Addr "); Serial.print(addr, HEX); Serial.print(" = "); Serial.println(val, HEX);
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
void cmd_print_all(int arg_cnt, char **args)
{
  accel.printAllRegister();
}

/**************************************************************************/
// calculate elapsed time. this takes into account rollover.
/**************************************************************************/
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
