/*                                                                                              |
 * University of Minnesota - Twin Cities                                                        |
 * AEM 4490 - Introduction to Aerospace Topics                                                  |
 * GLEAM Project - Device                                                                       |
 * Author: Joe Poeppel - poepp027@umn.edu                                                       |
 * Date: 2/23/2021                                                                              |
 *                                                                                              | 
 * XBee Series 3 Mesh Network: Measurement Device for GLEAM Project                             |
 * This software is to be placed on the measurement units that are to collect data, log         |
 * the data to an SD card, and send data to the Ground Unit Reciever (GUR) once a request is    |
 * detected from the Ground Unit Transmitter (GUT).                                             |
 ----------------------------------------------------------------------------------------------*/
 
// SENSORs, SD, and I2C LIBRARIES
#include <Wire.h>                                       // I2C  library                    - Should already be on your computer as a part of the IDE download
#include <SPI.h>                                        // SPI  library                    - Should already be on your computer as a part of the IDE download
#include <SD.h>                                         // SD   library                    - Should already be on your computer as a part of the IDE download
#include <XBee.h>                                       // XBee library                    - https://github.com/MNSGC-Ballooning/XBee
#include <SparkFunLSM9DS1.h>                            // IMU  library                    - https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library
#include <SFE_MicroOLED.h>                              // OLED library                    - https://learn.sparkfun.com/tutorials/micro-oled-breakout-hookup-guide/arduino-library-download-install-and-test

// SENSOR LIBRARIES FOR YOUR RESPECTIVE I2C SENSOR
#include <Adafruit_VEML6070.h>                          // VEML6070 I2C Sensor             - Go to Sketch > Include Library > Manage Libraries... > (search "VEML6070" and install library...click 'Install all' if needed)
#include <Adafruit_VEML7700.h>                          // VEML7700 I2C Sensor             - Go to Sketch > Include Library > Manage Libraries... > (search "VEML7700" and install library...click 'Install all' if needed)
#include <Adafruit_AS726x.h>                            // AS7252   I2C Sensor             - Go to Sketch > Include Library > Manage Libraries... > (search "AS726x"   and install library...click 'Install all' if needed)
#include <Adafruit_SI1145.h>                            // SI1145   I2C Sensor             - Go to Sketch > Include Library > Manage Libraries... > (search "SI1145"   and install library...click 'Install all' if needed)

                                                        // Note: Don't worry if your library name doesn't turn orange. As long as you have this library downloaded and  
                                                        //       located in the proper folder, Arduino (in this case Teensyduino) will be able to properly use the library.
                                                        
// UNIT VARIABLE
String Unit = "B1";                                     // *** MUST CHANGE THIS TO YOUR ASSIGNED UNIT (A1, A2, ..., B1, B2, ... C1, C2, ..., D1, D2, ...)

// SD CARD VARIABLES
#define chipSelect BUILTIN_SDCARD                       // Using built in chipselect on Teensy 3.5
File datalog;                                           // File object to be opened, written to, and closed
char filename[] = "GLM00.csv";                          // File name as will be seen on SD card -- can have maximum of 99 files on SD card ('GLM00' -> 'GLM99')
bool sdActive = false;                                  // Boolean to check if there are any available filenames left to be used 

// XBEE SERIAL DECLARATION
#define XBeeSerial Serial5                              // xBee serial TX5 and RX5 on pins 33 and 34, respectively 

// MICRO_OLED SETTINGS
#define PIN_RESET 9                                     // The SFE_MicroOLED.h library assumes a reset pin is necessary.The Qwiic OLED has RST hard-wired, so pick an arbitrary IO pin that is not being used.
#define DC_JUMPER 1                                     // The DC_JUMPER is the I2C Address Select jumper. Set to 1 if the jumper is open (default)

// TEENSY 3.5 PIN CONNECTIONS
#define THERM A17                                       // Analog pin 17 for thermistor
#define PHOTO A16                                       // Analog pin 16 for photoresistor
#define ANALOGSENSOR A18                                // Analog pin 18 for respective analog sensor
#define LED1 24                                         // Digital pin for LED1
#define LED2 25                                         // Digital pin for LED2
#define buttonPin 38                                    // Digital pin for button

// BAUD RATES
#define SERIAL_BAUD_RATE 9600                           // Baud rate of serial monitor
#define XBEE_BAUD_RATE 115200                           // Baud rate of xBee serial

// OBJECT DELCARATIONS
MicroOLED oled(PIN_RESET, DC_JUMPER);                   // OLED Object
LSM9DS1 imu;                                            // IMU  Object
XBee xBee = XBee(&XBeeSerial);                          // xBee Object

// OBJECT DECLARATION FOR YOUR RESPECTIVE I2C SENSOR
Adafruit_VEML6070 veml6070 = Adafruit_VEML6070();       // VEML6070 Object
Adafruit_VEML7700 veml7700 = Adafruit_VEML7700();       // VEML7700 Object
Adafruit_AS726x AS;                                     // AS7262   Object
Adafruit_SI1145 SI = Adafruit_SI1145();                 // SI1145   Object

// GENERAL VARIABLES
String Data;                                            // String containing all data from sensors
String spacer = ", ";                                   // Used to make creating Data and SDData strings quicker and more organized 
String endline = "\n";                                  // Used to make creating Data and SDData strings quicker and more organized 
String delimiter = "E";                                 // Used to make creating Data and SDData strings quicker and more organized - delimiter must be an uppercase 'E' in order for the GUR to properly read your data
String timer;                                           // Hours/Minutes/Seconds
String header;                                          // Used as first row of .csv file to distinguish logged data
int delayLength = 0;                                    // Delay length (in milliseconds) between each main loop iteration
int dataLogs = 0;                                       // Number of times data has been logged to SD card
float setUpTime;                                        // Used to start logging at t = 0 seconds

// IMU VARIABLES
float magnetometer[3];                                  // Three element array for holding magnetometer values in x, y, z directions, respectively
float accelerometer[3];                                 // Three element array for holding acceleration values in x, y, z directions, respectively
float gyroscope[3];                                     // Three element array for holding gyroscope    values in x, y, z directions, respectively
float roll;                                             // Roll of IMU
float pitch;                                            // Pitch of IMU
float heading;                                          // Heading of IMU
float DECLINATION = -7.22;                              // Declination of Minneapolis, MN

// THERMISTOR CONSTANTS and VARIABLES
int analogResolutionBits = 14;                          // Sets the size (in bits) of the value returned by analogRead(). It defaults to 10 bits (returns values between 0-1023), but we can request greater accuracy by using more bits
int analogResolutionVals = pow(2,analogResolutionBits); // Maximum possible analog resolution with base-2 math
int adcMax = pow(2, analogResolutionBits) - 1.0;        // Maximum possible analog reading from thermistor (1024 values between 0-1023) (adc stands for analog-to-digital converter converts the analog signal from the sensor into a digital signal within the Teensy 3.5) 
float A = 0.001125308852122;                            // Steinhart - Hart equation constant
float B = 0.000234711863267;                            // Steinhart - Hart equation constant
float C = 0.000000085663516;                            // Steinhart - Hart equation constant
float R1 = 10000;                                       // Resistor value (in ohms) being used in series with the thermistor
float ThermistorData;                                   // Analog value reading from thermistor
float logR;                                             // Necessary log value used in Steinhart - Hart equation
float Tinv;                                             // Temperature in degrees kelvin inverted
float T;                                                // Temperature in degress kelvin
float currentTempC;                                     // Temperature in degrees celcius
float currentTempF;                                     // Temperature in degrees fahrenheit

// PHOTOERSISTOR CONSTANTS and VARIABLES
float PhotoresistorData;                                // Analog value reading from photoresistor

// ANALOG SENSOR VARIABLES
float AnalogSensorData;                                 // Analog sensor data
String AnalogSensorBeingUsed = "GUVA-S12SD";            // *** MUST CHANGE THIS TO YOUR ASSIGNED ANALOG SENSOR ("GUVA-S12SD", "ALS-PT19")

// GENERAL I2C SENSOR VARIABLES
String I2CSensorBeingUsed = "AS7262";                   // *** MUST CHANGE THIS TO YOUR ASSIGNED I2C SENSOR ("VEML6070", "VEML7700", "AS7262", "SI1145")

// VEML6070 SENSOR VARIABLES
float VEML6070Data;                                     

// VEML7700 SENSOR VARIABLES
float VEML7700Lux;                                      
float VEML7700White;
float VEML7700RawALS;

// AS7262 SENSOR VARIABLES
uint8_t  AS7262STemperature;
uint16_t AS7262SensorValues[AS726x_NUM_CHANNELS];
uint16_t AS7262Violet;
uint16_t AS7262Blue;
uint16_t AS7262Green; 
uint16_t AS7262Yellow; 
uint16_t AS7262Orange; 
uint16_t AS7262Red; 

// SI1145 SENSOR VARIABLES
float SI1145Visible;
float SI1145IR;
float SI1145UV;

// XBEE VARIABLES
String xBeeString;                                      // String to throw away incoming data request ("!") from transmitter 
String xBeeHeader;                                      // Header string to be sent via the xBee to the GUR
String xBeeData;                                        // Data string to be sent via the xBee to the GUR
bool xBeeHeaderSent = false;                            // Bool to allow the header to only be sent once

// OLED VARIABLES
int button = 0;                                         // Variable that allows user to push button to change what is displayed on the OLED
float displayTimer = 0;                                 // Variable used in allowing the OLED to display a message for a given amount of time without stopping the rest of the program

// LED VARIABLES
int ledsONorOFF = 1;                                    // *** Set = 1 to enable LEDs; Set = 0 to disable LEDs

// SETUP FUNCTIONS
void startupProcedure();                                // Startup function which carries out the setup of the Serials, OLED, LEDs, and Sensors
void OLEDSetup();                                       // Setup function for OLED
void IMUSetup();                                        // Setup function for IMU
void I2CSensorSetup(String I2Ssensor);                  // Setup function for your respective I2C sensor
void SDSetup();                                         // Setup function for SD card

// UPDATE FUNCTIONS
void updateIMU();                                       // Update function for IMU
void updateThermistor();                                // Update function for thermistor
void updatePhotoresistor();                             // Update function for photoresistor
void updateAnalogSensor();                              // Update function for your respective Analog sensor
void updateI2CSensor(String I2CSensor);                 // Update function for your respective I2C sensor
void updateDataStrings(String I2CSensor);               // Update function for data string that will be sent via xBee and to the SD
void updateSD();                                        // Update function for SD (logging data to SD card)
void updateXBee(String text);                           // Update function for xBee (sending xBeeData string over xBee when data request is received)
void updateOLED(String text);                           // Update function for OLED

// EXTRA USER FUNCTIONS
void getHeader(String I2CSensor, String AnalogSensor);  // Functioon that sets up head based on the Analog and I2C sensors being used
void updateTimer();                                     // Sets the 'timer' variable
void getDisplay();                                      // Allows user to switch what is displayed on OLED by pushing a button
void blinkLED(int x);                                   // Function that blinks LED(x) once -> x = 1 for LED1 or x = 2 for LED2
void blinkLEDs(int x);                                  // Function that blinks both LEDs 'x' number of times -> x = 2 for two blinks

// ADDITIONAL FUNCITON OF IMU
void updateRoll();                                      // Updates 'roll' variable
void updatePitch();                                     // Updates 'pitch' variable
void updateHeading();                                   // Updates 'heading' variable

void setup() {      
                                  
  startupProcedure();
  
}


void loop() {
  
  updateTimer();
  updateIMU();
  updateThermistor();
  updatePhotoresistor();
  updateAnalogSensor();
  updateI2CSensor(I2CSensorBeingUsed);
  updateDataStrings(I2CSensorBeingUsed);
  updateSD(Data);
  updateXBee(xBeeData);
  getDisplay();
  delay(delayLength);
  
}



/////////////////// SETUP FUNCTION DEFINITIONS ///////////////////
void startupProcedure() {
  int delayTime = 250;
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  for(int i = 0; i <4; i++) {
    blinkLED(1);
    blinkLED(2);
  }
  delay(delayTime);
  
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("Serial monitor connected!");
  blinkLEDs(2);
  delay(delayTime);

  Serial.print("Starting OLED setup...       ");
  OLEDSetup();
  Serial.println("complete!");
  blinkLEDs(2);
  delay(delayTime);

  Serial.print("Display button setup...      ");
  pinMode(buttonPin, INPUT);
  updateOLED("Button\nonline!");
  Serial.println("complete!");
  blinkLEDs(2);
  delay(delayTime);
  
  Serial.print("Connecting xBee serial...    ");
  XBeeSerial.begin(XBEE_BAUD_RATE);
  Serial.println("complete!");
  updateOLED("xBee\nonline!");
  blinkLEDs(2);
  delay(delayTime);

  Serial.print("Starting IMU setup...        ");
  IMUSetup();
  updateIMU();
  Serial.println("complete!");
  blinkLEDs(2);
  delay(delayTime);

  Serial.print("Starting I2C sensor setup... ");
  I2CSensorSetup(I2CSensorBeingUsed);
  Serial.println("complete!");
  updateOLED("I2C sensoronline!");
  blinkLEDs(2);
  delay(delayTime);

  Serial.print("Starting SD setup... ");
  SDSetup();
  getHeader(I2CSensorBeingUsed, AnalogSensorBeingUsed);
  blinkLEDs(2);
  delay(delayTime);

  Serial.println("Beginning logging sequence:");
  updateOLED("Data log\nbeginning");
  updateSD(header);
  blinkLEDs(3);
  delay(delayTime);

  setUpTime = millis();
}

void OLEDSetup() {
  Wire.begin();
  oled.begin();
  oled.clear(ALL);
  oled.display();
  oled.clear(PAGE);
  randomSeed(analogRead(A0) + analogRead(A1));
  delay(250);
  updateOLED("OLED\nonline!");
}

void IMUSetup() {
  Wire.begin();
  if(!imu.begin()) {
    Serial.println("Failed to communicate with IMU.");
    delay(500);
  }

  else{
    updateOLED("IMU\nonline!");
    delay(500);
  }
}

void I2CSensorSetup(String I2CSensor) {
  
  if (I2CSensor == "VEML6070") {
    veml6070.begin(VEML6070_1_T);
  }

  else if (I2CSensor == "VEML7700") {
    veml7700.begin();
    veml7700.setGain(VEML7700_GAIN_1);
    veml7700.setIntegrationTime(VEML7700_IT_800MS);
    veml7700.setLowThreshold(10000);
    veml7700.setHighThreshold(20000);
    veml7700.interruptEnable(true);
  }

  else if (I2CSensor == "AS7262") {
    AS.begin();
  }

  else if (I2CSensor == "SI1145") {
    SI.begin();
  }

  else {
    Serial.println("Error - (in I2CSensorSetup()): Please change the variable 'I2CSensorBeingUsed' to one of the four I2C sensor options and reset system!");
    Serial.println("Exiting program!");
    updateOLED("Error!\n\nExiting\nprogram!");
    while(1) {}   
  }
}

void SDSetup(){
  pinMode(chipSelect, OUTPUT);
  if(!SD.begin(chipSelect)) {
    Serial.println("SD card failed, or not present");
    updateOLED("SD Card\nfailed\nor\nnot\npresent");
    delay(1000); 
  }
  else {
    Serial.print("        card initialized! \nCreating File...             ");
    for (byte i = 0; i<100; i++) {
      filename[3] = '0' + i/10;
      filename[4] = '0' + i%10;
      if(!SD.exists(filename)) {
        datalog = SD.open(filename, FILE_WRITE);
        sdActive = true;
        Serial.println("complete!");
        Serial.println("Logging to: " + String(filename));
        updateOLED("SD\nonline!");
        delay(500);
        updateOLED("Logging:\n\n" + String(filename));
        delay(500);
        break;}
      }

      if(!sdActive) {
        Serial.println("No available file names; clear SD card to enable logging");
        updateOLED("Clear SD!");
        delay(500);
    }
  }
}



/////////////////// UPDATE FUNCTION DEFINITIONS ///////////////////
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

  updateRoll();
  updatePitch();
  updateHeading();
}

void updateThermistor() {
  analogReadResolution(analogResolutionBits);
  ThermistorData = analogRead(THERM);
  logR = log(((adcMax/ThermistorData)-1)*R1);
  Tinv = A + B * logR + C * logR * logR * logR;         // Steinhart - Hart equation
  T = 1/Tinv;
  currentTempC = T - 273.15;
  currentTempF = currentTempC * 9 / 5 + 32;
}

void updatePhotoresistor() {
  PhotoresistorData = analogRead(PHOTO);                // May need proper formula to get useful values -- otherwise will just see varying analog values
}

void updateAnalogSensor() {
  AnalogSensorData = analogRead(ANALOGSENSOR);          // May need proper formula to get useful values -- otherwise will just see varying analog values
}

void updateI2CSensor(String I2CSensor) {

  if (I2CSensor == "VEML6070") {
    VEML6070Data = veml6070.readUV();
  }

  else if (I2CSensor == "VEML7700") {
    VEML7700Lux = veml7700.readLux();
    VEML7700White = veml7700.readWhite();
    VEML7700RawALS = veml7700.readALS(); 
  }

  else if (I2CSensor == "AS7262") {
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
  }

  else if (I2CSensor == "SI1145") {
    SI1145Visible = SI.readVisible();
    SI1145IR = SI.readIR();
    SI1145UV = SI.readUV() / 1000;
  }

  else {
    Serial.println("Error - (in updateI2CSensor()): Please change the variable 'I2CSensorBeingUsed' to one of the four I2C sensor options!");
    Serial.println("Exiting program!");
    updateOLED("Error!\n\nExiting\nprogram!");
    while(1) {}
  }
}

void updateDataStrings(String I2CSensor) {
  
  Data = Unit + spacer + String((millis() - setUpTime)/1000) + spacer + String(magnetometer[0]) + spacer + String(magnetometer[1]) + spacer + String(magnetometer[2]) + spacer;
  Data = Data + String(accelerometer[0]) + spacer + String(accelerometer[1]) + spacer + String(accelerometer[2]) + spacer;
  Data = Data + String(gyroscope[0]) + spacer + String(gyroscope[1]) + spacer + String(gyroscope[2]) + spacer;
  Data = Data + String(currentTempF) + spacer + String(currentTempC) + spacer;
  Data = Data + String(PhotoresistorData) + spacer + String(AnalogSensorData) + spacer;
  
  xBeeData = Unit + spacer + String((millis() - setUpTime)/1000) + spacer + String(currentTempF) + spacer + String(currentTempC) + spacer + String(PhotoresistorData) + spacer + String(AnalogSensorData) + spacer;

  if(I2CSensor == "VEML6070") {
    Data = Data + String(VEML6070Data);
    xBeeData = xBeeData + String(VEML6070Data) + delimiter;
  }
  else if(I2CSensor == "VEML7700") { 
    Data = Data + String(VEML7700Lux) + spacer + String(VEML7700White) + spacer + String (VEML7700RawALS);
    xBeeData = xBeeData + String(VEML7700Lux) + spacer + String(VEML7700White) + spacer + String (VEML7700RawALS) + delimiter;
  }
  else if(I2CSensor == "AS7262") {
    Data = Data + String(AS7262STemperature) + spacer + String(AS7262Violet) + spacer + String(AS7262Blue) + spacer + String(AS7262Green) + spacer + String(AS7262Yellow) + spacer + String(AS7262Orange) + spacer + String(AS7262Red);
    xBeeData = xBeeData + String(AS7262STemperature) + spacer + String(AS7262Violet) + spacer + String(AS7262Blue) + spacer + String(AS7262Green) + spacer + String(AS7262Yellow) + spacer + String(AS7262Orange) + spacer + String(AS7262Red) + delimiter;
  }
  else if(I2CSensor == "SI1145") {
    Data = Data + String(SI1145Visible) + spacer + String(SI1145IR) + spacer + String(SI1145UV);
    xBeeData = xBeeData + String(SI1145Visible) + spacer + String(SI1145IR) + spacer + String(SI1145UV) + delimiter;
  }
  else {
    Serial.println("Error - (in updateDataStrings()): Please change the variable 'I2CSensorBeingUsed' to one of the four I2C sensor options!");
    Serial.println("Exiting program!");
    updateOLED("Error!\n\nExiting\nprogram!");
    while(1) {}
  }                  
}

void updateSD(String text) {                                
  datalog = SD.open(filename, FILE_WRITE);
  datalog.println(text);
  datalog.close();
  blinkLED(1);
  dataLogs++;
  Serial.println(text);
}

void updateXBee(String text) {
 if(xBee.available()) {
  Serial.println("Data request received from GUT!\n");
  xBeeString = xBee.read();
   if (xBeeHeaderSent == false) {
    xBee.print(xBeeHeader);
    Serial.println(xBeeHeader);
    Serial.println("Header sent to GUR!\n");
    xBeeHeaderSent = true;
   }
  else {
    xBee.print(text);
    Serial.println(text);
    Serial.println("Data sent to GUR via xBee!\n");
    updateOLED("Request\nfrom GUT!\nData sent\nvia xBee\nto GUR!");
    displayTimer = millis();
    blinkLED(2);
    }
  }
}

void updateOLED(String text) {
  oled.clear(PAGE);
  oled.setFontType(0);
  oled.setCursor(0,0);
  oled.println(text);
  oled.display();
}



/////////////////// EXTRA USER FUNCTIONS ///////////////////
void getHeader(String I2CSensor, String AnalogSensor) {
  
  header = "Unit, Time, MagX, MagY, MagZ, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, TempF, TempC, Photo, " + AnalogSensor + spacer;
  xBeeHeader = String(Unit) + spacer +  "Time, TempF, TempC, Photo, " + AnalogSensor + spacer;
  
  if(I2CSensor == "VEML6070") {
    header = header + String("VEML6070");
    xBeeHeader = xBeeHeader + String("VEML6070") + delimiter;
  }

  else if(I2CSensor == "VEML7700") {
    header = header + String("VEML7700_Lux") + spacer + String("VEML7700_White") + String ("VEML7700_RawALS");
    xBeeHeader = xBeeHeader + String("VEML7700_Lux") + spacer + String("VEML7700_White") + String ("VEML7700_RawALS") + delimiter;
  }

  else if(I2CSensor == "AS7262") {
    header = header + String("AS7262_Temperature") + spacer + String("AS7262_Violet") + spacer + String("AS7262_Blue") + spacer + String("AS7262_Green") + spacer + String("AS7262_Yellow") + spacer + String("AS7262_Orange") + spacer + String("AS7262_Red");
    xBeeHeader = xBeeHeader + String("AS7262_Temperature") + spacer + String("AS7262_Violet") + spacer + String("AS7262_Blue") + spacer + String("AS7262_Green") + spacer + String("AS7262_Yellow") + spacer + String("AS7262_Orange") + spacer + String("AS7262_Red") + delimiter;
  }

  else if(I2CSensor == "SI1145") {
    header = header + String("SI1145_Visible") + spacer + String("SI1145_IR") + spacer + String("SI1145_UV");
    xBeeHeader = xBeeHeader + String("SI1145_Visible") + spacer + String("SI1145_IR") + spacer + String("SI1145_UV") + delimiter;
  }

  else {
    Serial.println("Error - (in getHeader()): Please change the variable 'I2CSensorBeingUsed' to one of the four I2C sensor options!");
    Serial.println("Exiting program!");
    while(1==1) {}
  }
}

void getDisplay() {

  if (millis() - displayTimer > 2000){                   // Allows the OLED to keep the message "Request from GUT! Data sent via xBee to GUR!" displayed for a longer period of time while still running code
  
    if(digitalRead(buttonPin) == HIGH) {
       button++;
       if(button > 5) button = 0;
     }
     
   switch(button) {
      case 0: updateOLED(String(currentTempF)+ " F" + endline + String(currentTempC) + " C" + endline + String(PhotoresistorData) + endline + String(AnalogSensorData)); break;
      case 1: updateOLED("Mag Vals" + endline + endline + "X: " + String(magnetometer[0]) + endline + "Y: " + String(magnetometer[1]) + endline + "Z: " + String(magnetometer[2])); break;
      case 2: updateOLED("Gryo Vals" + endline + endline + "X: " + String(gyroscope[0]) + endline + "Y: " + String(gyroscope[1]) + endline + "Z: " + String(gyroscope[2])); break;
      case 3: updateOLED("Accel Vals" + endline + "X: " + String(accelerometer[0]) + endline + "Y: " + String(accelerometer[1]) + endline + "Z: " + String(accelerometer[2])); break;
      case 4: updateOLED("R: " + String(roll) + endline + endline + "P: " + String(pitch) + endline + endline + "H: " + String(heading)); break;
      case 5: updateOLED(String(filename) + "\nLog: " + String(dataLogs) + "   " + endline + timer); break;
     }
  }
} 

void updateTimer() {
  int time = (millis() - setUpTime)/1000;                                     // Time we are converting
  int hr = time/3600;                                                         // Number of hours
  int mins = (time-hr*3600)/60;                                               // Remove the number of hours and calculate the minutes.
  int sec = time-hr*3600-mins*60;                                            // Remove the number of hours and minutes, leaving only seconds.
  timer = (String(hr) + ":" + String(mins) + ":" + String(sec));             // Converts to HH:MM:SS string
}

void blinkLED(int x) {
  if (ledsONorOFF == 1) {
    if (x == 1) {
      digitalWrite(LED1, HIGH);
      delay(50);
      digitalWrite(LED1, LOW);
    }
  
    if (x == 2) {
      digitalWrite(LED2, HIGH);
      delay(50);
      digitalWrite(LED2, LOW);
    }
  
    if(x != 1 && x != 2) {
      {Serial.println("Error: 'x' value inputted into function 'blinkLED(x)' not valid! - - - Please use x = 1 or x = 2");}
      while(1){}
    }
  }
}

void blinkLEDs(int x) {

  if (ledsONorOFF == 1) {
    for (int i = 0; i<x; i++) {
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
      delay(150);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      delay(150);
    }
  }
}


  // ADDITIONAL FUNCTIONS OF IMU 
void updateRoll() {

 roll = atan2(accelerometer[1], accelerometer[2]);
 roll *= 180.0 / PI;
  
}

void updatePitch() {

  pitch = atan2(-accelerometer[0], sqrt(accelerometer[1]*accelerometer[1] + accelerometer[2]*accelerometer[2]));
  pitch *= 180.0 / PI;
  
}

void updateHeading() {
  
  if (magnetometer[1] == 0) {
    heading = (magnetometer[0] < 0) ? PI : 0;
  }
  
  else {
    heading = atan2(magnetometer[0], magnetometer[1]);
    heading -= DECLINATION * PI / 180;
  }
  
   if(heading > PI) { heading -= (2 * PI); }
   
   else if (heading < -PI) { heading += (2 * PI);}
   
    heading *= 180.0 / PI;
}
    
