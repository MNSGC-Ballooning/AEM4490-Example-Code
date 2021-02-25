/*                                                                                        |
 * University of Minnesota - Twin Cities                                                  |      |
 * AEM 4490 - Introduction to Aerospace Topics                                            |      |
 * GLEAM Project - Mesaurement Device component testing                                   |                                       |
 * Author: Joe Poeppel - poepp027@umn.edu                                                 |          |
 * Date: 2/24/2021                                                                        |
                                                                                          |
 * This program is to be used when testing component connections with the GLEAM DEVICE.   |
 * Simply change the number in line 99 in order to change which component is to be tested.|
 *                                                                                        | 
 * You may see values being printed to the serial monitor even if your component isn't    |
 * connected! Please use your discretion when determining if your component is working.   |
 ----------------------------------------------------------------------------------------*/
 

#include <Wire.h>                                       // I2C  library                    - Should already be on your computer as a part of the IDE download
#include <SparkFunLSM9DS1.h>                            // IMU  library                    - https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
#include <SFE_MicroOLED.h>                              // OLED library                    - https://learn.sparkfun.com/tutorials/micro-oled-breakout-hookup-guide/arduino-library-download-install-and-test


// SENSOR LIBRARIES FOR YOUR RESPECTIVE I2C SENSOR
#include <Adafruit_VEML6070.h>                          // VEML6070 I2C Sensor             - Go to Sketch > Include Library > Manage Libraries... > (search "VEML6070" and install library...click 'Install all' if needed)
#include <Adafruit_VEML7700.h>                          // VEML7700 I2C Sensor             - Go to Sketch > Include Library > Manage Libraries... > (search "VEML7700" and install library...click 'Install all' if needed)
#include <Adafruit_AS726x.h>                            // AS7252   I2C Sensor             - Go to Sketch > Include Library > Manage Libraries... > (search "AS726x"   and install library...click 'Install all' if needed)
#include <Adafruit_SI1145.h>                            // SI1145   I2C Sensor             - Go to Sketch > Include Library > Manage Libraries... > (search "SI1145"   and install library...click 'Install all' if needed)

// MICRO_OLED SETTINGS
#define PIN_RESET 9                                     // The SFE_MicroOLED.h library assumes a reset pin is necessary.The Qwiic OLED has RST hard-wired, so pick an arbitrary IO pin that is not being used.
#define DC_JUMPER 1                                     // The DC_JUMPER is the I2C Address Select jumper. Set to 1 if the jumper is open (default)

// OBJECT DELCARATIONS
MicroOLED oled(PIN_RESET, DC_JUMPER);                   // OLED Object
LSM9DS1 imu;                                            // IMU  Object

// OBJECT DECLARATION FOR YOUR RESPECTIVE I2C SENSOR
Adafruit_VEML6070 veml6070 = Adafruit_VEML6070();       // VEML6070 Object
Adafruit_VEML7700 veml7700 = Adafruit_VEML7700();       // VEML7700 Object
Adafruit_AS726x AS;                                     // AS7262   Object
Adafruit_SI1145 SI = Adafruit_SI1145();                 // SI1145   Object

// GENERAL VARIABLES
String Data = "";
String spacer = ", ";

// IMU VARIABLES
float magnetometer[3];                                  // Three element array for holding magnetometer values in x, y, z directions, respectively
float accelerometer[3];                                 // Three element array for holding acceleration values in x, y, z directions, respectively
float gyroscope[3];                                     // Three element array for holding gyroscope    values in x, y, z directions, respectively
float roll;                                             // Roll of IMU
float pitch;                                            // Pitch of IMU
float heading;                                          // Heading of IMU
float DECLINATION = -7.22;                              // Declination of Minneapolis, MN

// VEML6070 SENSOR VARIABLES
float VEML6070Data = 0;                                     

// VEML7700 SENSOR VARIABLES
float VEML7700Lux = 0;                                      
float VEML7700White = 0;
float VEML7700RawALS = 0;

// AS7262 SENSOR VARIABLES
uint8_t  AS7262STemperature = 0;
uint16_t AS7262SensorValues[AS726x_NUM_CHANNELS];
uint16_t AS7262Violet = 0;
uint16_t AS7262Blue = 0;
uint16_t AS7262Green = 0; 
uint16_t AS7262Yellow = 0; 
uint16_t AS7262Orange = 0; 
uint16_t AS7262Red = 0; 

// SI1145 SENSOR VARIABLES
float SI1145Visible = 0;
float SI1145IR = 0;
float SI1145UV = 0;

// SETUP FUNCTIONS
void setupIMU();
void setupOLED();
void setupVEML6070();
void setupVEML7700();
void setupAS7262();
void setupSI1145();

// UPDATE FUNCTIONS
void updateIMU();
void updateOLED();
void updateVEML6070();
void updateVEML7700();
void updateAS7262();
void updateSI1145();
void printData();





// COMPONENT BEING TESTED
int componentBeingTested = 1;                       // Options: OLED = 1, IMU = 2, VEML6070 = 3, VEML7700 = 4, AS7262 = 5, SI1145 = 6





void setup() {

  Serial.begin(9600);
  if(componentBeingTested < 1 || componentBeingTested > 6) {
    Serial.println("Error: 'componentBeingTested' variable not set to a listed component value, please check this and try again!");
  }
  switch(componentBeingTested) {
     case 1: setupOLED(); break;
     case 2: setupIMU(); break;
     case 3: setupVEML6070(); break;
     case 4: setupVEML7700(); break;
     case 5: setupAS7262(); break;
     case 6: setupSI1145(); break;
  }
}

void loop() {
  
   switch(componentBeingTested) {
     case 1: updateOLED(); break;
     case 2: updateIMU(); break;
     case 3: updateVEML6070(); break;
     case 4: updateVEML7700(); break;
     case 5: updateAS7262(); break;
     case 6: updateSI1145(); break;
  }
  printData();
  delay(100);
}


/////////////////// SETUP FUNCTION DEFINITIONS ///////////////////
void setupOLED() {
  Wire.begin();
  if(!oled.begin()) {
    Serial.println("Failed to communicate with OLED");
    delay(500);
  }
  
  else {
    oled.clear(ALL);
    oled.display();
    oled.clear(PAGE);
    randomSeed(analogRead(A0) + analogRead(A1));
    delay(250);
    Serial.println("OLED online!");
    delay(500);
  }
}


void setupIMU() {
  Wire.begin();
  if(!imu.begin()) {
    Serial.println("Failed to communicate with IMU.");
    delay(500);
  }

  else{
    Serial.println("IMU online!");
    delay(500);
  }
}


void setupVEML6070() {
  veml6070.begin(VEML6070_1_T);
}

void setupVEML7700() {
  veml7700.begin();
  veml7700.setGain(VEML7700_GAIN_1);
  veml7700.setIntegrationTime(VEML7700_IT_800MS);
  veml7700.setLowThreshold(10000);
  veml7700.setHighThreshold(20000);
  veml7700.interruptEnable(true);
}

void setupAS7262() {
  AS.begin();
}

void setupSI1145() {
  SI.begin();
}

/////////////////// UPDATE FUNCTION DEFINITIONS ///////////////////
void updateOLED() {
  oled.clear(PAGE);
  oled.setFontType(0);
  oled.setCursor(0,0);
  oled.print("Hello!\nProgram\nruntime:\n" + String(millis()/1000));
  oled.display();
}


void updateIMU() {

  if (imu.magAvailable()) imu.readMag();
  if (imu.accelAvailable()) imu.readAccel();
  if (imu.gyroAvailable()) imu.readGyro();

  magnetometer[0] = imu.calcMag(imu.mx);
  magnetometer[1] = imu.calcMag(imu.my);
  magnetometer[2] = imu.calcMag(imu.mz);
  accelerometer[0] = imu.calcAccel(imu.ax);
  accelerometer[1] = imu.calcAccel(imu.ay);
  accelerometer[2] = imu.calcAccel(imu.az);
  gyroscope[0] = imu.calcGyro(imu.gx);
  gyroscope[1] = imu.calcGyro(imu.gy);
  gyroscope[2] = imu.calcGyro(imu.gz);


  Data = String(magnetometer[0]) + spacer + String(magnetometer[1]) + spacer + String(magnetometer[2]) + spacer;
  Data = Data + String(accelerometer[0]) + spacer + String(accelerometer[1]) + spacer + String(accelerometer[2]) + spacer;
  Data = Data + String(gyroscope[0]) + spacer + String(gyroscope[1]) + spacer + String(gyroscope[2]);
}


void updateVEML6070() {
  VEML6070Data = veml6070.readUV();
  Data = String(VEML6070Data);
}

void updateVEML7700() {
  VEML7700Lux = veml7700.readLux();
  VEML7700White = veml7700.readWhite();
  VEML7700RawALS = veml7700.readALS();
  Data = String(VEML7700Lux) + spacer + String(VEML7700White) + spacer + String (VEML7700RawALS);
}

void updateAS7262() {
    AS7262STemperature = AS.readTemperature();
    AS.startMeasurement();
    bool ready = false;
    while(!ready){
      delay(5);
      ready = AS.dataReady();
    }

    AS.readRawValues(AS7262SensorValues);
    
    AS7262Violet = AS7262SensorValues[AS726x_VIOLET];
    AS7262Blue = AS7262SensorValues[AS726x_BLUE];
    AS7262Green = AS7262SensorValues[AS726x_GREEN];
    AS7262Yellow = AS7262SensorValues[AS726x_YELLOW];
    AS7262Orange = AS7262SensorValues[AS726x_ORANGE];
    AS7262Red = AS7262SensorValues[AS726x_RED];

    Data = String(AS7262STemperature) + spacer + String(AS7262Violet) + spacer + String(AS7262Blue) + spacer + String(AS7262Green) + spacer + String(AS7262Yellow) + spacer + String(AS7262Orange) + spacer + String(AS7262Red);
}

void updateSI1145() {
   SI1145Visible = SI.readVisible();
   SI1145IR = SI.readIR();
   SI1145UV = SI.readUV() / 100.0;

   Data = String(SI1145Visible) + spacer + String(SI1145IR) + spacer + String(SI1145UV);
}

void printData() {

  Serial.println(Data);
  Data = "";
  
}
