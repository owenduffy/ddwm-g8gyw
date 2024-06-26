// https://g8gyw.github.io/
//
// Copyright (c) 2021 Mike G8GYW
// Revision 2022 Owen Duffy
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This program is designed to run on an Atmega328P with 8MHz clock.

// ATMEGA FUSE VALUES
// L:FF   H:D3   E:FD //NOTE: this specifies an external crystal as used in a Chinese copy
// L:E2   H:D3   E:FD //NOTE: this specifies the internal RC oscillator as used in G8GYW's article

#define VERSION "0.03"
#define READ_INTEEPROM
//#define WRITE_INTEEPROM
//#define CALMODE

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int barh=SCREEN_HEIGHT/2;
int basey=display.height()-1;
int tickh=barh/4;

#if defined(__AVR_ATmega328P__)
HardwareSerial &MySerial=Serial;
#include <EEPROM.h>
#endif

struct{
  uint16_t ever=0;
  #if defined(CALMODE)
  uint16_t avgn=4*2;
  #else
  uint16_t avgn=2;
  #endif
  float pmin=0.05;
  int adcoffsadj=0;
  uint16_t flags=0;
  float vreff=1.0718*(18+10)/10; //MeasuredAdcRef*(R3+R4)/R4
  float vrefr=1.1225/1.1173*1.0718*(18+10)/10; //MeasuredAdcRef*(R7+R8)/R8 //adjusted to calibrate to vreff
  float a=0.039358125429379,b=0.92716713641334,c=0.91574481497507,d=0.11403999912754;
} parms;
int adcpf=A0,adcpr=A1; // select the input pin for the detector
unsigned AdcAccumulator; // variable to accumulate the value coming from the sensor
float calfactorf,calfactorr;

float Vfwd;   // Forward voltage
float Vrev;   // Reverse voltage
float Pfwd;   // Forward power
float Prev;   // Reverse power
float SWR;    // VSWR
float rho;  // Reflection coefficient
float Vbat;   // Battery voltage

void setup(){
  float adcref;
  long adcfs;
  int ever;

  #if defined(__AVR_ATmega328P__)
  pinMode(A0,INPUT);
  analogReference(INTERNAL);
  adcref=1.10;
  adcfs=1024;
  #endif
  analogRead(A2); //Read ADC2
  delay (500); // Allow ADC to settle
  float vbat=analogRead(A2); //Read ADC again
  vbat=4.9*(vbat + 0.5)/(float)adcfs*adcref; //Calculate battery voltage scaled by R9 & R10
  // Display startup screen
  MySerial.begin(9600);
  MySerial.println(F("Starting..."));
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //Initialize with the I2C address 0x3C.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("Directional wattmeter");
  display.setCursor(0, 16);
  display.setTextSize(2);
  display.print(" 12W MAX");
  display.setTextSize(1);
  display.display();
  delay(1000);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("ddwm ver: ");
  display.println(VERSION);
  display.print("vbat: ");
  display.println(vbat,1);
  display.display();
  delay(1000);

  #if defined(WRITE_INTEEPROM)
  MySerial.println(F("writing eeprom..."));
  EEPROM.put(0x0,parms);
  MySerial.println(F("done"));
  while(1);
  #endif
  #if defined(READ_INTEEPROM)
  //get the data block from eeprom
  ever=EEPROM.read(0);
  if(ever!=0){
    MySerial.print(F("Unsupported EEPROM version: "));
    MySerial.println(ever);
    while(1);
  }
  EEPROM.get(0,parms);
  MySerial.println(F("Read EEPROM."));
  #endif

  calfactorf=parms.vreff/parms.avgn/(float)adcfs;
  calfactorr=parms.vrefr/parms.avgn/(float)adcfs;
}

void loop(){
  int prec;
  int i;
  float vinf,pwrf,vinr,pwrr,dbm;
  long AdcAccumulatorf=0,AdcAccumulatorr=0;

  for(i=parms.avgn;i--;){
    // read the value from the detector
    AdcAccumulatorf+=analogRead(adcpf);
    AdcAccumulatorr+=analogRead(adcpr);
    delay(100);
    }
  // calculate average v
  vinf=((float)AdcAccumulatorf+(float)parms.adcoffsadj*(float)parms.avgn)*calfactorf;
  vinr=((float)AdcAccumulatorr+(float)parms.adcoffsadj*(float)parms.avgn)*calfactorr;
  #if defined(CALMODE)
  pwrf=vinf;
  pwrr=vinr;
  #else
  pwrf=parms.a+parms.b*vinf+parms.c*pow(vinf,2)+parms.d*pow(vinf,3);
  pwrr=parms.a+parms.b*vinr+parms.c*pow(vinr,2)+parms.d*pow(vinr,3);
  #endif
/*
  MySerial.print(F("AdcAccumulatorf: "));
  MySerial.print(AdcAccumulatorf);
  MySerial.print(F(", vinf: "));
  MySerial.print(vinf,10);
  MySerial.print(F(", pwrf: "));
  MySerial.println(pwrf,10);
  MySerial.print(F("AdcAccumulatorr: "));
  MySerial.print(AdcAccumulatorr);
  MySerial.print(F(", vinr: "));
  MySerial.print(vinr,10);
  MySerial.print(F(", pwrr: "));
  MySerial.println(pwrr,10);
*/
  //display Power and VSWR
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor (0,0);
  #if defined(CALMODE)
  display.println(pwrf,8);
  display.println(pwrr,8);
  #else
  if(pwrf<parms.pmin){
    pwrf=0.0;
    dbm=-99;
    prec=3;
    }
  else{
    dbm=10*log10(pwrf/0.001);
    prec=4-floor(dbm/10);
    }
  if(pwrf>=parms.pmin){
    prec+=1;
    display.print(pwrf,prec);
    display.print(F("W"));
    }
  else
    display.print(F("***W"));

  if(pwrf>parms.pmin && pwrr>parms.pmin){
    rho=sqrt(pwrr/pwrf); // Calculate reflection coefficient
  //  MySerial.print(F(", rho: "));
  //  MySerial.println(rho,10);
    rho=constrain(rho,0,0.99999);
    SWR=(1+rho)/(1-rho);
    SWR=constrain(SWR,1,99.9);
//    MySerial.print(F(", swr: "));
//    MySerial.println(SWR,10);
    int w=((SWR-1)*32)+0.5+65;
    // Draw filled part of bar starting from left of screen:
    display.fillRect(65,display.height()-1-barh,w,barh,1);
    display.fillRect(w+1,display.height()-barh-1,display.width()-w,barh,0);
    for(int i=80;i<128;i=i+16)
      display.fillRect(i,basey-barh+3*tickh,1,barh-3*tickh,i>w?WHITE:BLACK);
    for(int i=96;i<128;i=i+32)
      display.fillRect(i,basey-barh+2*tickh,1,barh-2*tickh,i>w?WHITE:BLACK);

    display.setCursor(0,17);
    display.print(SWR,1); // Display VSWR to one decimal place
    }
#endif
  display.display();
  }
