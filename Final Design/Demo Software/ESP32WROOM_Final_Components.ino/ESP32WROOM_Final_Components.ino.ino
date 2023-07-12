/*
  Name: 
  Date: 
  Author: Daan Heetkamp
  
  Description:
  
  Revision: 
  0.1
*/
 
// include libraries
#include <Wire.h>

// MPU6050
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Include SparkFun MAX3010x Pulse and Proximity Sensor Library
#include "MAX30105.h"
#include "heartRate.h"

#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library

// ----- Declare Constants ----- 
int buttonPin = 0;
int batMonPin = 35;
int vibrationPin = 25;
int buzzerPin = 12;

// Initialize sensor object for:
  // Heartrate
  MAX30105 particleSensor;

  // MPU6050
  Adafruit_MPU6050 mpu;

  //Screen
  #define TFT_GREY 0x5AEB

  TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

// ----- Declare Objects ----- 
// ----- Declare subroutines and/or functions ----- 
//MPU6050
void initMPU6050();
void checkMPU6050();

//Heartrate Sensor
void initHeartRate();
void checkHeartRate();

// ----- Declare Global Variables ----- 
//Heartrate
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
    
bool heartFound = false;
bool MPUFound = false;

//Screen
float sx = 0, sy = 1, mx = 1, my = 0, hx = -1, hy = 0;    // Saved H, M, S x & y multipliers
float sdeg=0, mdeg=0, hdeg=0;
uint16_t osx=120, osy=120, omx=120, omy=120, ohx=120, ohy=120;  // Saved H, M, S x & y coords
uint16_t x0=0, x1=0, yy0=0, yy1=0;
uint32_t targetTime = 0;                    // for next 1 second timeout

static uint8_t conv2d(const char* p); // Forward declaration needed for IDE 1.6.x
uint8_t hh=conv2d(__TIME__), mm=conv2d(__TIME__+3), ss=conv2d(__TIME__+6);  // Get H, M, S from compile time

bool initial = 1;

// Setup
void setup() 
{   
  Serial.begin(115200);

  //Wire.begin(20, 21); //SDA, SCL
  Wire.begin(21, 22); //SDA, SCL
  //Wire.begin();

  // Initialize MPU6050
  initMPU6050();
  Serial.println("INIT MPU done");
  
  // Initialize Heartrate Sensor
  initHeartRate();
  Serial.println("INIT HR DONE");


  pinMode(vibrationPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(batMonPin, INPUT);

  tft.init();
  tft.setRotation(0);
  
  //tft.fillScreen(TFT_BLACK);
  //tft.fillScreen(TFT_RED);
  //tft.fillScreen(TFT_GREEN);
  //tft.fillScreen(TFT_BLUE);
  //tft.fillScreen(TFT_BLACK);
  tft.fillScreen(TFT_GREY);
  
  tft.setTextColor(TFT_WHITE, TFT_GREY);  // Adding a background colour erases previous text automatically
  
  // Draw clock face
  tft.fillCircle(120, 120, 118, TFT_GREEN);
  tft.fillCircle(120, 120, 110, TFT_BLACK);

  // Draw 12 lines
  for(int i = 0; i<360; i+= 30) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*114+120;
    yy0 = sy*114+120;
    x1 = sx*100+120;
    yy1 = sy*100+120;

    tft.drawLine(x0, yy0, x1, yy1, TFT_GREEN);
  }

  // Draw 60 dots
  for(int i = 0; i<360; i+= 6) {
    sx = cos((i-90)*0.0174532925);
    sy = sin((i-90)*0.0174532925);
    x0 = sx*102+120;
    yy0 = sy*102+120;
    // Draw minute markers
    tft.drawPixel(x0, yy0, TFT_WHITE);
    
    // Draw main quadrant dots
    if(i==0 || i==180) tft.fillCircle(x0, yy0, 2, TFT_WHITE);
    if(i==90 || i==270) tft.fillCircle(x0, yy0, 2, TFT_WHITE);
  }

  tft.fillCircle(120, 121, 3, TFT_WHITE);

  targetTime = millis() + 1000; 

  delay(100);
  Serial.println("Setup Done");
}

// Main loop
void loop() {  

  //Serial.println(digitalRead(buttonPin));
  if (digitalRead(buttonPin) == LOW)
  {
    Serial.println("Button Pressed");
    digitalWrite(vibrationPin, HIGH);

    delay(2000);
    
    digitalWrite(vibrationPin, LOW);

    tone(buzzerPin, 100, 200);
    delay(200);
    tone(buzzerPin, 500, 200);
    delay(200);
    tone(buzzerPin, 1000, 200);
    delay(200);

    noTone(buzzerPin);
  }

  int batteryValue = analogRead(batMonPin);
  //Serial.println(batteryValue);
  batteryValue = (map(batteryValue, 0, 4095, 0, 100));
  //Serial.println(batteryValue);

  //Measure MPU
  checkMPU6050();
  //Serial.println("Searching MPU Done");

  //Measure HR
  checkHeartRate();
  //Serial.println("Searching HR done");

  if (targetTime < millis()) {
    targetTime += 1000;
    ss++;              // Advance second
    if (ss==60) {
      ss=0;
      mm++;            // Advance minute
      if(mm>59) {
        mm=0;
        hh++;          // Advance hour
        if (hh>23) {
          hh=0;
        }
      }
    }

    // Draw text at position 120,260 using fonts 4
    // Only font numbers 2,4,6,7 are valid. Font 6 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : . - a p m
    // Font 7 is a 7 segment font and only contains characters [space] 0 1 2 3 4 5 6 7 8 9 : .
    tft.drawCentreString("BPM:  ",100,180,2);
    tft.drawCentreString("X: ",100,40,2);
    tft.drawCentreString("Y: ",100,60,2);
  
    char cstr[16];
    itoa(beatAvg, cstr, 10);
    //String stringOne = String(beatAvg, DEC);        // using an int and a base
    tft.drawCentreString(cstr,130,180,2);


    // Pre-compute hand degrees, x & y coords for a fast screen update
    sdeg = ss*6;                  // 0-59 -> 0-354
    mdeg = mm*6+sdeg*0.01666667;  // 0-59 -> 0-360 - includes seconds
    hdeg = hh*30+mdeg*0.0833333;  // 0-11 -> 0-360 - includes minutes and seconds
    hx = cos((hdeg-90)*0.0174532925);    
    hy = sin((hdeg-90)*0.0174532925);
    mx = cos((mdeg-90)*0.0174532925);    
    my = sin((mdeg-90)*0.0174532925);
    sx = cos((sdeg-90)*0.0174532925);    
    sy = sin((sdeg-90)*0.0174532925);

    if (ss==0 || initial) {
      initial = 0;
      // Erase hour and minute hand positions every minute
      tft.drawLine(ohx, ohy, 120, 121, TFT_BLACK);
      ohx = hx*62+121;    
      ohy = hy*62+121;
      tft.drawLine(omx, omy, 120, 121, TFT_BLACK);
      omx = mx*84+120;    
      omy = my*84+121;
    }

      // Redraw new hand positions, hour and minute hands not erased here to avoid flicker
      tft.drawLine(osx, osy, 120, 121, TFT_BLACK);
      osx = sx*90+121;    
      osy = sy*90+121;
      tft.drawLine(osx, osy, 120, 121, TFT_RED);
      tft.drawLine(ohx, ohy, 120, 121, TFT_WHITE);
      tft.drawLine(omx, omy, 120, 121, TFT_WHITE);
      tft.drawLine(osx, osy, 120, 121, TFT_RED);

    tft.fillCircle(120, 121, 3, TFT_RED);
  }
  
}



/******************FUNCTIONS**********************/
static uint8_t conv2d(const char* p) {
  uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

void initMPU6050(){
  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  else{
    MPUFound = true;

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
    }
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
    }
  }
}

void initHeartRate(){

  // Initialize Heartrate Sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
  }
  else{

    Serial.println("MAX30105 Found!");
    heartFound = true;

    Serial.println("Place your index finger on the sensor with steady pressure.");

    particleSensor.setup(); //Configure sensor with default settings
    particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
    particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  }
}


void checkMPU6050(){

  //Measure MPU
  if(MPUFound)
  {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    char cstr[16];
    itoa(a.acceleration.x, cstr, 10);
    tft.drawCentreString(cstr,130,40,2);

    itoa(a.acceleration.y, cstr, 10);
    tft.drawCentreString(cstr,130,60,2);

    /* Print out the values */
    Serial.print("Acceleration X: ");
    Serial.print(a.acceleration.x);
    Serial.print(", Y: ");
    Serial.print(a.acceleration.y);
    Serial.print(", Z: ");
    Serial.print(a.acceleration.z);
    Serial.println(" m/s^2");

    Serial.print("Rotation X: ");
    Serial.print(g.gyro.x);
    Serial.print(", Y: ");
    Serial.print(g.gyro.y);
    Serial.print(", Z: ");
    Serial.print(g.gyro.z);
    Serial.println(" rad/s");

    Serial.print("Temperature: ");
    Serial.print(temp.temperature);
    Serial.println(" degC");

    Serial.println("");
  }
}

void checkHeartRate(){

  //Measure HR
  if(heartFound)
  {
    Serial.println("Searching HR");
    long irValue = particleSensor.getIR();

    if (checkForBeat(irValue) == true)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);
    Serial.println();

    if (irValue < 50000)
    {
      Serial.print(" No finger?");
      Serial.println();
    }
  }
}


