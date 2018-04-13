/*
#include <Servo.h>

#include <bitswap.h>
#include <chipsets.h>
#include <color.h>
#include <colorpalettes.h>
#include <colorutils.h>
#include <controller.h>
#include <cpp_compat.h>
#include <dmx.h>
#include <FastLED.h>
#include <fastled_config.h>
#include <fastled_delay.h>
#include <fastled_progmem.h>
#include <fastpin.h>
#include <fastspi.h>
#include <fastspi_bitbang.h>
#include <fastspi_dma.h>
#include <fastspi_nop.h>
#include <fastspi_ref.h>
#include <fastspi_types.h>
#include <hsv2rgb.h>
#include <led_sysdefs.h>
#include <lib8tion.h>
#include <noise.h>
#include <pixelset.h>
#include <pixeltypes.h>
#include <platforms.h>
#include <power_mgt.h>
#include"AirQuality.h"
*/
//#include"Arduino.h"
//AirQuality airqualitysensor;
int current_quality = -1;
/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/

// the setup function runs once when you press reset or power the board
/***************************************************
  DFPlayer - A Mini MP3 Player For Arduino
  <https://www.dfrobot.com/index.php?route=product/product&product_id=1121>

 ***************************************************
  This example shows the basic function of library for DFPlayer.

  Created 2016-12-07
  By [Angelo qiao](Angelo.qiao@dfrobot.com)

  GNU Lesser General Public License.
  See <http://www.gnu.org/licenses/> for details.
  All above must be included in any redistribution
 ****************************************************/

/***********Notice and Trouble shooting***************
  1.Connection and Diagram can be found here
  <https://www.dfrobot.com/wiki/index.php/DFPlayer_Mini_SKU:DFR0299#Connection_Diagram>
  2.This code is tested on Arduino Uno, Leonardo, Mega boards.
 ****************************************************/



#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
/*
  #include "FastLED.h"

  #define LED_PIN     5
  #define NUM_LEDS    4
  #define BRIGHTNESS  64
  #define LED_TYPE    WS2811
  #define COLOR_ORDER GRB
  CRGB leds[NUM_LEDS];

  #define UPDATES_PER_SECOND 100
  CRGBPalette16 currentPalette;
  TBlendType    currentBlending;

  extern CRGBPalette16 myRedWhiteBluePalette;
  extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;
*/
//#include <Adafruit_NeoPixel.h>
//#ifdef __AVR__
//#include <avr/power.h>
//#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
//#define PIN            5

// How many NeoPixels are attached to the Arduino?
//#define NUMPIXELS      4

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
//Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int r_lum = 0;
int g_lum = 0;
int b_lum = 0;
//dgt11
#include "DHT.h"

#define DHTPIN 4     // what digital pin we're connected to

// Uncomment whatever type you're using!
#define DHTTYPE DHT11   // DHT 11
//#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
//#define DHTTYPE DHT21   // DHT 21 (AM2301)

// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO

SoftwareSerial DFP(2, 3); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);



SoftwareSerial BT(7, 8); 
char val; 
int brightness = 0;
int fadeAmount = 5;
int delayDuration = 30;
int R = 11;
int G = 10;
int B = 9;
void setup() {

      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

  
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  //pixels.begin(); // This initializes the NeoPixel library.

  //pixels.show();
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);

  Serial.begin(9600);   // 嚙踝蕭�謆�嚙踝��嚙踐�蕭蹎ｇ蕭嚙質嚙�
  Serial.println("BT is ready!");

  // ���蕭謍喉蕭��蕭謕���蕭��蕭嚙質謍湛蕭賹蕭嚙�
  // ���蕭謚恃�HC-05��嚙踝��蕭嚙踝蕭38400
  BT.begin(9600);


  DFP.begin(9600);

  DFP.listen();
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(DFP)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    //while (true);
  }
  Serial.println(F("DFPlayer Mini online."));

  //myDFPlayer.volume(1);  //Set volume value. From 0 to 30
  //myDFPlayer.loop(1);
  //set led
  analogWrite(R, 0);
  analogWrite(G, 0);
  analogWrite(B, 0);
  //pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // Moderately bright green color.
  //pixels.setPixelColor(1, pixels.Color(0, 255, 0)); // Moderately bright green color.
  //pixels.setPixelColor(2, pixels.Color(0, 0, 255)); // Moderately bright green color.
  //pixels.setPixelColor(3, pixels.Color(255, 255, 255)); // Moderately bright green color.
  //pixels.show();
  //airqualitysensor.init(14);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}
char command[8];
int mode = 0; //0 for initail,1 for song ,2 for volumn, 3 for R ,4 for G,5 forB
// the loop function runs over and over again forever
int size = 0;
/*
void ws2812b(int r, int g, int b, int led) {
  pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // Moderately bright green color.
  pixels.setPixelColor(1, pixels.Color(0, 0, 0)); // Moderately bright green color.
  pixels.setPixelColor(2, pixels.Color(0, 0, 0)); // Moderately bright green color.
  pixels.setPixelColor(3, pixels.Color(0, 0, 0)); // Moderately bright green color.
  pixels.setPixelColor(4, pixels.Color(0, 0, 0)); // Moderately bright green color.
  pixels.setPixelColor(5, pixels.Color(0, 0, 0)); // Moderately bright green color.
  pixels.setPixelColor(6, pixels.Color(0, 0, 0)); // Moderately bright green color.
  pixels.setPixelColor(7, pixels.Color(0, 0, 0)); // Moderately bright green color.
  pixels.setPixelColor(led, pixels.Color(r, g, b)); // Moderately bright green color.
  pixels.show();

}*/
void change_status(int num) {
  Serial.println(F("Change level to"));
  Serial.println(num);
  if (mode == 2) {
    myDFPlayer.volume(num * 3);
  } else if (mode == 1) {
    myDFPlayer.loop(num + 1);
  } else if (mode == 3) {
    analogWrite(R,  (num * 28));
    r_lum = num;
  } else if (mode == 4) {
    analogWrite(G, (num * 28));
    g_lum = num;
  } else if (mode == 5) {
    analogWrite(B, (num * 28));
    b_lum = num;
  }

}



void loop() {
  delay(20);
  static unsigned int led = 0;
  led++;
  BT.listen();

  
  if (Serial.available()) {
    val = Serial.read();
    // Serial.print(val);
    BT.print(val);
  }

 


  if (BT.available()) {
    val = BT.read();
    if (val == 'V' || val == 'v') {
      Serial.print("volumn\r\n");
      //myDFPlayer.volumeUp();
      mode = 2;
    }
    else if (val == 's' || val == 'S') {
      Serial.print("song\r\n");
      //myDFPlayer.volumeDown();
      mode = 1;
    }
    else if (val == 'r' || val == 'R') {
      Serial.print("Red\r\n");
      //myDFPlayer.volumeDown();
      mode = 3;
    } else if (val == 'g' || val == 'G') {
      Serial.print("Green\r\n");
      //myDFPlayer.volumeDown();
      mode = 4;
    } else if (val == 'b' || val == 'B') {
      Serial.print("Blue\r\n");
      //myDFPlayer.volumeDown();
      mode = 5;

    } else if (val == 'h' || val == 'H') {
      mode = 6;
      float h = dht.readHumidity();
      Serial.print("Humidity: ");
      Serial.print(h);
      Serial.print(" %\t");

      BT.print("H");
      BT.print(h);
      BT.print("H");

    } else if (val == 't' || val == 'T') {
      mode = 7;
      float t = dht.readTemperature();
      Serial.print("Temperature: ");
      Serial.print(t);
      Serial.print(" *C ");
      BT.print("T");
      BT.print(t);
      BT.print("T");
 
    } else if (val == 'a' || val == 'A') {
      current_quality = analogRead(A0);
      /*if (current_quality >= 0)// if a valid data returned.
      {
        if (current_quality == 0)
          Serial.println("High pollution! Force signal active");
        else if (current_quality == 1)
          Serial.println("High pollution!");
        else if (current_quality == 2)
          Serial.println("Low pollution!");
        else if (current_quality == 3)
          Serial.println("Fresh air");
      }*/
     Serial.print(current_quality);
     BT.print("A");
     BT.print(current_quality);
     BT.print("A");
    }else if(val=='c'||val=='C'){
      current_quality = analogRead(A1);
     
     Serial.print("co");
     BT.print("C");
     BT.print(current_quality);
     BT.print("C");
      
      }else if (val == 'w' || val == 'W') {
      accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\n");
        Serial.print(ay); Serial.print("\n");
        Serial.print(az); Serial.print("\n");
      
      BT.print("W");
        BT.print(ax); BT.print("\n");
        BT.print(ay); BT.print("\n");
        BT.print(az); BT.print("\n");
      BT.print("W");
 
    }
    int num = val;
    num = num - 30;
    if (num >= 18 && num <= 27)
      change_status(num - 18);


  }
  //ws2812b(r_lum, g_lum, b_lum, led % 8);
  //delay(100);
}
/*
ISR(TIMER1_OVF_vect)
{
  if (airqualitysensor.counter == 61) //set 2 seconds as a detected duty
  {

    airqualitysensor.last_vol = airqualitysensor.first_vol;
    airqualitysensor.first_vol = analogRead(A0);
    airqualitysensor.counter = 0;
    airqualitysensor.timer_index = 1;
    PORTB = PORTB ^ 0x20;
  }
  else
  {
    airqualitysensor.counter++;
  }
}*/


