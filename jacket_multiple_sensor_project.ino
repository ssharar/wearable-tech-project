/*
This code was written by Shaeeta Sharar
  many of its contents have been taken from or based off of example codes provided by the open source Adafruit libraries

*/


#include <CapPin.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

//Declerations:
//  delaring all sensors and lights
#define PIN 6    //defines neopixel pin placement

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0();    //defines gyroscope

CapPin cPin_10 = CapPin(10);   // read pin 10    defines touch sensor 1, the light sensor
CapPin cPin_9 = CapPin(9);   // read pin 9       defines touch sensor 2, the gyroscope sensor
CapPin cPin_1 = CapPin(1);   // read tx          defines touch sensor 3, the "control" sensor (lights all neopixels)

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);    //defines light sensor

Adafruit_NeoPixel strip = Adafruit_NeoPixel(40, PIN, NEO_GRB + NEO_KHZ800);    //defines neopixel strip

bool bool10 = false;    //booleans used to control touch sensors, named after their respective pins
bool bool9 = false;
bool bool1 = false;

//list of colors used to generate random colors
//                                  R   G   B
uint8_t myFavoriteColors[][3] = {{202,   133, 255},   // sunset purple
                                 {255,   133,   169}, // sunset pink
                                 {253, 128, 055},     // sunset orange
                                 {255, 255, 255},     // white
                                 {163, 243, 255},     // beach teal
                                 {102, 250, 120},     // beach green
                                 {124, 128, 242},     // beach blue
                               };
// don't edit the line below
#define FAVCOLORS sizeof(myFavoriteColors) / 3

//adjusts motion threshold
#define MOVE_THRESHOLD 400


void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
}

void setup() {
  //Initializing the sensors/neopixels
  if(!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    //Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

 /* while (!Serial); // flora & leonardo
  
  Serial.begin(9600);
  Serial.println("LSM raw read demo");
  */
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    //Serial.println("Oops ... unable to initialize the LSM9DS0. Check your wiring!");
    while (1);
  }
  //Serial.println("Found LSM9DS0 9DOF");
  //Serial.println("");
  //Serial.println("");
  
  strip.begin();
  strip.setBrightness(30);
  strip.show();

}

void loop() {

  //these store the readings from the touch sensors
  delay(1);
  long total1 = 0;
  long start = millis();
  long total10 =  cPin_10.readPin(2000);
  long total9 =  cPin_9.readPin(2000);
  long totaltx =  cPin_1.readPin(2000);
  
  //initializing an event
  sensors_event_t event;
  tsl.getEvent(&event);  
  
  //if the touch sensor for the light sensor is activated (or has been active), set all other touch sensor booleans to inactive,
  //  check the brightness and if it's dark enough, light the neopixels using flash random, otherwise turn them off
  if ((total10 > 10) || bool10)
    {
       bool10 = true;
       bool9 = false;
       bool1 = false; 
       if (event.light <= 10)
       {
          flashRandom(20, 1);  // first number is 'wait' delay, shorter num == shorter twinkle
          flashRandom(20, 3);  // second number is how many neopixels to simultaneously light up
          flashRandom(20, 2);
       }
       else
       {
         off();
       }
       delay(250);
       
    }
 
 //if the touch sensor for the gyroscope is activated (or has been active), set all other touch sensor booleans to inactive,
 //  read in to see how much motion is detected, if the amount exceeds the previously set threshold, light the neopixels using flash random, otherwise turn them off
  if ((total9 > 10) || bool9)
    {
       bool9 = true;
       bool10 = false;
       bool1 = false;
       
         lsm.read();
        
          double storedVector = lsm.accelData.x*lsm.accelData.x;
          storedVector += lsm.accelData.y*lsm.accelData.y;
          storedVector += lsm.accelData.z*lsm.accelData.z;
          storedVector = sqrt(storedVector);
          
          // wait a bit
          delay(100);
          
          // get new data!
          lsm.read();
          double newVector = lsm.accelData.x*lsm.accelData.x;
          newVector += lsm.accelData.y*lsm.accelData.y;
          newVector += lsm.accelData.z*lsm.accelData.z;
          newVector = sqrt(newVector);
          
          delay(100);
       
       if (abs(newVector - storedVector) > MOVE_THRESHOLD) {
        //Serial.println("Twinkle!");
        flashRandom(20, 1);  // first number is 'wait' delay, shorter num == shorter twinkle
        flashRandom(20, 3);  // second number is how many neopixels to simultaneously light up
        flashRandom(20, 2);
       }
      
       delay(500);
    }

 //if the touch sensor for the control is activated (or has been active), set all other touch sensor booleans to inactive,
 //  light the neopixels using setColor() and let them blink
  if ((totaltx > 10) || bool1)
    {
       bool1 = true;
       bool10 = false;
       bool9 = false;
       setColor();
       Blink();
       delay(5);
    }

}

//function used to turn off every neopixel in the strip
void off(){
  
  for (int i=0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i,strip.Color(0,0,0));
  }
  
  strip.show();

}

//function used to twinkle the neopixels
//taken from Becky Stern's Sparkle Skirt tutorial with minor adjustments
void flashRandom(int wait, uint8_t howmany) {
 
  for(uint16_t i=0; i<howmany; i++) {
    // pick a random favorite color!
    int c = random(FAVCOLORS);
    int red = myFavoriteColors[c][0];
    int green = myFavoriteColors[c][1];
    int blue = myFavoriteColors[c][2]; 
    
    // now we will 'fade' it in 5 steps
    for (int x=0; x < 5; x++) {
      int r = red * (x+1); r /= 5;
      int g = green * (x+1); g /= 5;
      int b = blue * (x+1); b /= 5;
      
      for (int i=0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
      }
      
      strip.show();
      delay(wait);
    }
    // & fade out in 5 steps
    for (int x=5; x >= 0; x--) {
      int r = red * x; r /= 5;
      int g = green * x; g /= 5;
      int b = blue * x; b /= 5;
      
      for (int i=0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(r, g, b));
      }
      
      strip.show();
      delay(wait);
    }
  }
  // LEDs will be off when done (they are faded to 0)
}

//function used set the color every neopixel in the strip
void setColor() {
 
    for (int i=0; i < strip.numPixels(); i++) {
      if (i == 3 || i == 7 ||  i == 9 ||  i == 16 ||  i == 21 ||  i == 23 ||  i == 13 || i == 30)
      {
        //Serial.println("Light Sensor Test");
        strip.setPixelColor(i,163, 243, 255);
      }
      else if (i == 34 || i == 17 ||  i == 19 ||  i == 26 ||  i == 11 ||  i == 8 ||  i == 10 || i == 5)
      {
         strip.setPixelColor(i,253, 128, 055);
      }
      else
      {
          strip.setPixelColor(i,200,200,200);
      }
    }
      
    strip.show();
    delay(500);
}

//function used to turn off every 3rd neopixel in the strip
void Blink() {
  for (int i=0; i < strip.numPixels(); i+=3) {
      strip.setPixelColor(i,0,0,0);
  }
  strip.show();
  delay(500);
}
