/*
   Measure distance for motor control

   Date : 31 july 2020
   Author : Sully VITRY

   Code based on SparkFun_VL6180X_demo.ino
*/

#include <TimeLib.h>
#include <Wire.h>  // For a connection via I2C using Wire includ only needed for Arduino 1.6.5 and earlier
#include "ESP32_Oled_Driver_for_SSD1306_display\SSD1306.h" // alias for `#include "SSD1306Wire.h"`

// Include the UI lib
#include "OLEDDisplayUi.h"

// Include custom images
#include "images.h"

#include <SparkFun_VL6180X.h>

#define LED_BUILTIN 2

/*const float GAIN_1    = 1.01;  // Actual ALS Gain of 1.01
  const float GAIN_1_25 = 1.28;  // Actual ALS Gain of 1.28
  const float GAIN_1_67 = 1.72;  // Actual ALS Gain of 1.72
  const float GAIN_2_5  = 2.6;   // Actual ALS Gain of 2.60
  const float GAIN_5    = 5.21;  // Actual ALS Gain of 5.21
  const float GAIN_10   = 10.32; // Actual ALS Gain of 10.32
  const float GAIN_20   = 20;    // Actual ALS Gain of 20
  const float GAIN_40   = 40;    // Actual ALS Gain of 40
*/
#define VL6180X_ADDRESS 0x29
#define TOF_POWER 14
#define TOF_GPIO0 12
#define TOF_GPIO1 27

#define TOF_MEAN  10

#define SDA_tof 21
#define SCL_tof 22

#define SDA_oled 5
#define SCL_oled 4

VL6180xIdentification identification;
VL6180x sensor(VL6180X_ADDRESS);

long int somme = 0;
int moyenne = 0;
int distance_mm = 0;
int int_mat[TOF_MEAN];
int i = 0;

//display variable
// Initialize the OLED display using Wire library
SSD1306Wire  display(0x3c, SDA_tof, SCL_tof);       //ttgo wifi oled 18650 battery
//SSD1306  display(0x3c, 21, 22);  //ttgo mini32 esp32
// SH1106 display(0x3c, D3, D5);

OLEDDisplayUi ui ( &display );

int screenW = 128; //128;
int screenH = 64;
int screenCenterX = screenW / 2;
int screenCenterY = ((screenH - 16) / 2) + 16; // top yellow part is 16 px height
int screenRadius = 23;

TwoWire I2C_oled = TwoWire(0);
TwoWire I2C_tof = TwoWire(1);



/*
  ___________                   __  .__
  \_   _____/_ __  ____   _____/  |_|__| ____   ____   ______
   |    __)|  |  \/    \_/ ___\   __\  |/  _ \ /    \ /  ___/
   |     \ |  |  /   |  \  \___|  | |  (  <_> )   |  \\___ \
   \___  / |____/|___|  /\___  >__| |__|\____/|___|  /____  >
       \/             \/     \/                    \/     \/
*/

void screenOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
}



void acc_Frame(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String acc_txt = "ToF";
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24); //24
  display->drawString(screenCenterX + x , screenCenterY + y, acc_txt );
}



void acc_Frame_status(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
  String acc_txt = String((int)distance_mm) + "°";
  display->setTextAlignment(TEXT_ALIGN_CENTER);
  display->setFont(ArialMT_Plain_24); //24
  display->drawString(screenCenterX + x , screenCenterY + y, acc_txt );
}



// This array keeps function pointers to all frames
// frames are the single views that slide in
//FrameCallback frames[] = { acc_Frame_status, acc_Frame };
FrameCallback frames[] = { acc_Frame_status , acc_Frame };

// how many frames are there?
int frameCount = 2;

// Overlays are statically drawn on top of a frame eg. a screen
OverlayCallback overlays[] = { screenOverlay };
int overlaysCount = 1;



/*
   _________       __
  /   _____/ _____/  |_ __ ________
  \_____  \_/ __ \   __\  |  \____ \
  /        \  ___/|  | |  |  /  |_> >
  /_______  /\___  >__| |____/|   __/
          \/     \/           |__|
*/
void setup() {

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, HIGH);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second*

  //power sensor
  pinMode(TOF_POWER, OUTPUT);
  digitalWrite(TOF_POWER, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(10);                       // wait for a second

  //GPIO0 high
  pinMode(TOF_GPIO0, OUTPUT);
  digitalWrite(TOF_GPIO0, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(10);                       // wait for a second

  //GPIO1 input intterrupt
  pinMode(TOF_GPIO1, INPUT);

  Serial.begin(115200); //Start Serial at 115200bps

  Wire.begin(SDA_tof,SCL_tof,100000); //Start I2C library
  Wire1.begin(SDA_oled,SCL_oled,100000); // there are no defined pins for the second peripheral.

//  I2C_oled.begin(SDA_oled, SCL_oled, 100000);
//  I2C_tof.begin(SDA_tof, SCL_tof, 100000);

  delay(100); // delay .1s

  sensor.getIdentification(&identification); // Retrieve manufacture info from device memory
  printIdentification(&identification); // Helper function to print all the Module information

  if (sensor.VL6180xInit() != 0) {
    Serial.println("FAILED TO INITALIZE"); //Initialize device and check for errors
    while(true);
  };

  sensor.VL6180xDefautSettings(); //Load default settings to get started.

  delay(1000); // delay 1s


  //setup OLED
  // The ESP is capable of rendering 60fps in 80Mhz mode
  // but that won't give you much time for anything else
  // run it in 160Mhz mode or just set it to 30 fps
  ui.setTargetFPS(60);

  // Customize the active and inactive symbol
  ui.setActiveSymbol(activeSymbol);
  ui.setInactiveSymbol(inactiveSymbol);

  // You can change this to
  // TOP, LEFT, BOTTOM, RIGHT
  ui.setIndicatorPosition(TOP);

  // Defines where the first frame is located in the bar.
  ui.setIndicatorDirection(LEFT_RIGHT);

  // You can change the transition that is used
  // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
  ui.setFrameAnimation(SLIDE_LEFT);

  // Add frames
  ui.setFrames(frames, frameCount);

  // Add overlays
  ui.setOverlays(overlays, overlaysCount);

  // Initialising the UI will init the display too.
  ui.init();

  display.flipScreenVertically();

}

void loop() {

  //Get Ambient Light level and report in LUX
  //Serial.print("Ambient Light Level (Lux) = ");

  //Input GAIN for light levels,
  // GAIN_20     // Actual ALS Gain of 20
  // GAIN_10     // Actual ALS Gain of 10.32
  // GAIN_5      // Actual ALS Gain of 5.21
  // GAIN_2_5    // Actual ALS Gain of 2.60
  // GAIN_1_67   // Actual ALS Gain of 1.72
  // GAIN_1_25   // Actual ALS Gain of 1.28
  // GAIN_1      // Actual ALS Gain of 1.01
  // GAIN_40     // Actual ALS Gain of 40

  //Serial.println( sensor.getAmbientLight(GAIN_1) );

  //Get Distance and report in mm
  //Serial.print("Distance measured (mm) = ");
  //Serial.println( sensor.getDistance() );

  int_mat[i % TOF_MEAN] = sensor.getDistance();

  somme += int_mat[i % TOF_MEAN];
  somme -= int_mat[(i + 1) % TOF_MEAN];
  i++;

  moyenne = somme / TOF_MEAN;
  distance_mm = moyenne;

  Serial.println( moyenne);


  delay(10);



  //oled refresh
  //changement animation
  int remainingTimeBudget = ui.update();

  if (remainingTimeBudget > 0) {
    // You can do some work here
    // Don't do stuff if you are below your
    // time budget.
    delay(remainingTimeBudget);
  }

};

void printIdentification(struct VL6180xIdentification *temp) {
  Serial.print("Model ID = ");
  Serial.println(temp->idModel);

  Serial.print("Model Rev = ");
  Serial.print(temp->idModelRevMajor);
  Serial.print(".");
  Serial.println(temp->idModelRevMinor);

  Serial.print("Module Rev = ");
  Serial.print(temp->idModuleRevMajor);
  Serial.print(".");
  Serial.println(temp->idModuleRevMinor);

  Serial.print("Manufacture Date = ");
  Serial.print((temp->idDate >> 3) & 0x001F);
  Serial.print("/");
  Serial.print((temp->idDate >> 8) & 0x000F);
  Serial.print("/1");
  Serial.print((temp->idDate >> 12) & 0x000F);
  Serial.print(" Phase: ");
  Serial.println(temp->idDate & 0x0007);

  Serial.print("Manufacture Time (s)= ");
  Serial.println(temp->idTime * 2);
  Serial.println();
  Serial.println();
}
