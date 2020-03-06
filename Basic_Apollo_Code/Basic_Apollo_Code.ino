/*Author: Billy Straub
 * Apollo dropper for use with Arduino MEGA microcontroller
 
 
 Please Read the Following:
 * NOTE: SPI and i2c connections are different on MEGA then on UNO as seen by the following connections.
 * Pinout for Arduino MEGA: https://duino4projects.com/wp-content/uploads/2013/04/ardunio_mega_pinout.jpg
 * 
 * Connections:
 * 
 * Use the following connections for the SD card reader (SPI connections):
 * CS -> 8
 * DI -> 51
 * SCK -> 52
 * DO -> 50
 * Leave pin 10 open as an output for the SD-card reader, it won't be wired to anything and should be left as an open port
 * 
 * Use the following connections for the XBee (Serial 1):
 * DOut -> 19
 * DIn -> 18
 * 
 * Use the following connections for the IMU (SDA and SCL i2c connections):
 * SDA -> 20
 * SCL -> 21
 * DON'T FORGET THAT THE IMU RUNS ON 3.3V NOT 5V! Don't break the IMU's please...
 * 
 * Pressure sensor connections:
 * Face the flat side towards yourself
 * Prong 1 -> A2
 * Prong 2 -> GND
 * Prong 3 -> 5V
 * Prong 4-6 -> (don't plug into anything, irrelevant, ignore)
 * 
 * Servo connections:
 * Orange data wire -> 6 (must be a PWM port)
 * White feedback wire -> A0
 */


#include <Servo.h>  //Servo library
#include <XBee.h>  //XBee library
#include <Wire.h>  //Wire library used by IMU
#include <SPI.h>  //SPI library used by SD-card reader
#include <SparkFunLSM9DS1.h>  //IMU library
#include <SD.h>  //SD card library


//Pin definitions
#define feedbackPin A0  //Pin for servo feedback
#define XBeeSerial Serial1  //RX(D_out) is pin 18, TX(D_in) is pin 19 on Arduino MEGA
#define SDchipSelect 8  //SD Pin Definition (must be pin 8 for sparkfun sd breakout board)
#define ServoPin 6  //Defines the term "ServoPin" as pin 6 (Servo pins must be attached to PWM ports)
#define presSensor A2  //Uses analog 2 for the pressure sensor data pin

//Variables for IMU stuff
#define PRINT_CALCULATED  
#define PRINT_SPEED 250 // 250 ms between prints
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//Objects
Servo myservo;  //States that "myservo" is a servo class object
XBee xBee = XBee(&XBeeSerial);  //States "xBee" is an XBee class object
LSM9DS1 imu;  //States "imu" is an LSM9DS1 IMU class object

//Variables
String message; // string that will be set to receive the commands from the XBee and hold them
int servoVal;   // integer that holds the servo position in degrees
float magX;     // magnetometer values
float magY;
float magZ;
float accX;     // acceleromter values
float accY;
float accZ;
float gyroX;    // gyroscopic values
float gyroY;
float gyroZ;
String IMUData; // string that will be set to hold all of the IMU data
int minDegrees; // servo calibration values
int maxDegrees;
float servoPos;
int minFeedback;
int maxFeedback;
int pressureSensor;  // presure sensor values
float pressureSensorVoltage;
float PressurePSI;
float PressureATM; 


File datalog;                     //File Object for datalogging
char filename[] = "Apol00.csv";   //Template for file name to save data in SD-card
bool SDactive = false;            //Used to check for SD card before attempting to log data



void setup() {

  Serial.begin(9600);  //begins serial communications that can be used for troubleshooting at a baud rate of 9600.
  XBeeSerial.begin(9600);  //start XBee communication
  Wire.begin(); 


  //Servo setup
  myservo.attach(ServoPin);  //Attaches the servo to whatever pin "ServoPin" is defined as
  calibrate(myservo, feedbackPin, 0, 180); // calibrates servo for 170 degree rotation. CALLS TO VOID CALIBRATE(), WHICH IS FOUND AT THE BOTTOM OF THE CODE.
  

  //SD Card Setup (Note: Don't need to change this code at all, but if you lengthen the file name at "char filename[] = "Apol00.csv";", you'll have to change the numbers in "filename[4] = '0' + i / 10;" and "filename[5] = '0' + i % 10;"
    pinMode(10, OUTPUT);                                      //Needed for SD library, regardless of shield used
    pinMode(SDchipSelect, OUTPUT);
    Serial.print("Initializing SD card...");
    xBee.print("Initializing SD card...");
    if (!SD.begin(SDchipSelect)){                              //Attempt to start SD communication
      Serial.println("Card not inserted or the card is broken.");          //Print out error if failed; remind user to check card
      xBee.println("Card not inserted or the card is broken.");
    }
    else {                                                    //If successful, attempt to create file
      Serial.println("Card initialized successfully.\nCreating File...");
      xBee.println("Card initialized successfully.\nCreating File...");
      for (byte i = 0; i < 100; i++) {                        //Can create up to 100 files with similar names, but numbered differently
        filename[4] = '0' + i / 10;
        filename[5] = '0' + i % 10;
        if (!SD.exists(filename)) {                           //If a given filename doesn't exist, it's available
          datalog = SD.open(filename, FILE_WRITE);            //Create file with that name
          SDactive = true;                                    //Activate SD logging since file creation was successful
          Serial.println("Logging to: " + String(filename));  //Tell user which file contains the data for this run of the program
          xBee.println("Logging to: " + String(filename));
          break;                                              //Exit the for loop now that we have a file
        }
      }
      if (!SDactive) {
      Serial.println("No available file names; clear SD card to enable logging");
      xBee.println("No available file names; clear SD card to enable logging");
      } 
    }


  //IMU possible start up error message
   if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");  //Prints error message on startup if the IMU is not wired correctly.
    xBee.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
  }


  delay(500);  //waits 500 ms

  //Prints header everytime on startup
  String headerLOG = "Mag(x), Mag(y), Mag(z), Acc(x), Acc(y), Acc(z), Gyro(x), Gyro(y), Gyro(z), Servo(deg), Pressure(atm), Time(ms)";  //Defines "headerLOG" as a string that contains the inscribed text.
  Serial.println(headerLOG);  //Prints the "headerLOG" to the serial monitor
  xBee.println(headerLOG);  //Prints the "headerLOG" to the xBee monitor
  if (SDactive) {
    datalog.println(headerLOG);  //Prints the "headerLOG" to the SD-card
    datalog.close();
  }


  delay(500);  //waits 500 ms
  
}



void loop() {

  //XBee data receiving loop
  if (xBee.available()) {
    message = xBee.readStringUntil('\n');  //reads each line of what is received from the xbee 
    myservo.write(message.toInt());  //takes xbee input command, coverts from string to integer, and then writes the degree value to the servo
  }


  //IMU loop (gets the new data every loop and redefines the variables and then puts them in the string "IMUData"
  if ( imu.gyroAvailable() ){
    imu.readGyro();
  }
  if ( imu.accelAvailable() ){
    imu.readAccel();
  }
  if ( imu.magAvailable() ){
    imu.readMag();
  }
  magX = imu.calcMag(imu.mx);
  magY = imu.calcMag(imu.my);
  magZ = imu.calcMag(imu.mz);
  accX = imu.calcAccel(imu.ax);
  accY = imu.calcAccel(imu.ay);
  accZ = imu.calcAccel(imu.az);
  gyroX = imu.calcMag(imu.gx);
  gyroY = imu.calcMag(imu.gy);
  gyroZ = imu.calcMag(imu.gz);
  IMUData = String(magX) + "," + String(magY) + "," + String(magZ) + "," + String(accX) + "," + String(accY) + "," + String(accZ) + "," + String(gyroX) + "," + String(gyroY) + "," + String(gyroZ);


  //Pressure Sensor conversions between analog value to atmosphere units
  pressureSensor = analogRead(presSensor);                 //Read the analog pin
  pressureSensorVoltage = pressureSensor * (5.0 / 1024);        //Convert the analog number to voltage
  PressurePSI = (pressureSensorVoltage - (0.1 * 5.0)) / (4.0 / 15.0);   //Convert the voltage to proper unitime
  PressureATM = PressurePSI / 14.696;                                //Convert psi to atm


  //Creates the string "dataLOG" and includes the device measurements listed, which is then printed to the serial monitor, the xBee monitor, and the SD-card. (Note: the getPos() is a function that leads to the void funtion at the bottom of the code, which runs said function.)
  String dataLOG = IMUData + "," + String(PressureATM, 3) + "," + getPos(feedbackPin) + "," + millis()/1000;
    Serial.println(dataLOG);
    xBee.println(dataLOG); 
    if (SDactive) {
      datalog = SD.open(filename, FILE_WRITE);
      datalog.println(dataLOG);                                
      datalog.close();                                      
    }


  delay(100);  //waits 100 ms before re-running the loop
 
}



void calibrate(Servo servo, int analogPin, int minPos, int maxPos) //When the "calibrate()" void function is ran in the setup, it leads here and runs this code.
{
  // Move to the minimum position and record the feedback values
  servo.write(minPos);
  minDegrees = minPos;
  delay(2000); // waits 2 sec to make sure it has time to get there and settle
  minFeedback = analogRead(analogPin);
  
  // Move to the maximum position and record the feedback values
  servo.write(maxPos);
  maxDegrees = maxPos;
  delay(2000); // waits 2 sec to make sure it has time to get there and settle
  maxFeedback = analogRead(analogPin);
}



int getPos(int analogPin)  //Called to in the "String dataLOG" by the function "getPos(feedbackPin)". Essentially, finds the angle that the servo is at and returns a value in degrees.
{
  return map(analogRead(analogPin), minFeedback, maxFeedback, minDegrees, maxDegrees);
}
