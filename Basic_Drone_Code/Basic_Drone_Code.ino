/*Author: Billy Straub
 * Drone sensor suite for use with Teensy 3.5 microcontroller
 
 
 Please Read the Following:
 * NOTE: To use the Arduino programming language with Teensy, download "Teensyduino" at https://www.pjrc.com/teensy/td_download.html
 * Pinout for Teensy 3.5: https://www.pjrc.com/teensy/card8a_rev2.pdf
 * 
 * Connections:
 * 
 * Dallas Temp sensor connections (One Wire bus):
 * https://docs.google.com/presentation/d/1STA4fFMdl7egD0iCnl-XdcBzythye2RyBZ2FBTOhtY0/edit#slide=id.g21cb78c36c_0_7
 * Both Dallas temp sensors are wired to 2.
 * Don't forget to add the 4.7 kOhm resistor between the power and data line, as shown by the link above.
 * 
 * Honeywell pressure sensor connections:
 * Face the barbed tip away from you.
 * Prong 1 -> GND
 * Prong 2 -> A0 (analog 0)
 * Prong 3 -> 5V
 * Prong 4 -> (doesn't plug into anything, irrelevant pin as far as we're concerned)
 * 
 * IMU connections:
 * https://docs.google.com/presentation/d/1STA4fFMdl7egD0iCnl-XdcBzythye2RyBZ2FBTOhtY0/edit#slide=id.g6ea3a5c718_0_75
 * SDA -> A4
 * SCL -> A5
 * 
 * Look at the your Teensy 3.5. Notice how theres an SD-card reader on it? That's your SD-card reader.
 * The built in SD-card reader is given the pin "BUILTIN_SDCARD"
 * Note: Leave pin 10 open as an output for the SD-card reader, it won't be wired to anything and should be left as an open port
 */

#include <Wire.h>  //Wire library used by IMU and Dallas temp sensors
#include <SPI.h>  //SPI library used by SD-card reader
#include <SparkFunLSM9DS1.h>  //IMU library
#include <SD.h>  //SD card library
#include <DallasTemperature.h>  //Dallas temp sensor library


//Pin definitions
#define SDchipSelect BUILTIN_SDCARD   //SD Pin Definition
#define PRINT_CALCULATED
#define PRINT_SPEED 250 // 250 ms between prints
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
#define Honeywell_1 A0                //Honeywell Pressure Sensor #1 Pin Definition
#define oneWireBus 2

//Objects
OneWire oneWire = OneWire(oneWireBus);  //Sets up a oneWire bus objects
DallasTemperature sensors = DallasTemperature(&oneWire);  //Create a DallasTemp object for both sensors
DeviceAddress inThermometer, outThermometer;  //Creates two temp sensor objects for each individual oneWire device
LSM9DS1 imu;  //States "imu" is an LSM9DS1 IMU class object

//Variables
float inTemp, outTemp;  // two temp sensor variables
int pressureSensor1;  // pressure sensor values
float pressureSensorVoltage1;
float PressurePSI1;
float PressureATM1;
float magX;     // magnetometer values
float magY;
float magZ;
String magData;  //string that will be set to hold all of the IMU magnetometer data


File datalog;                      //File Object for datalogging
char filename[] = "Drone00.csv";   //Template for file name to save data in SD-card
bool SDactive = false;             //Used to check for SD card before attempting to log data


void setup() {
    
    Serial.begin(115200);  //begins serial communications that can be used for troubleshooting at a baud rate of 115200.
    Wire.begin();
    sensors.begin();  //starts temp sensors


  //SD Card Setup (Note: Don't need to change this code at all, but if you lengthen the file name at "char filename[] = "Drone00.csv";", you'll have to change the numbers in "filename[5] = '0' + i / 10;" and "filename[6] = '0' + i % 10;"
    pinMode(10, OUTPUT);                                      //Needed for SD library, regardless of shield used
    pinMode(SDchipSelect, OUTPUT);
    Serial.print("Initializing SD card...");
    if (!SD.begin(SDchipSelect)){                              //Attempt to start SD communication
      Serial.println("Card not inserted or the card is broken.");          //Print out error if failed; remind user to check card
    }
    else {                                                    //If successful, attempt to create file
      Serial.println("Card initialized successfully.\nCreating File...");
      for (byte i = 0; i < 100; i++) {                        //Can create up to 100 files with similar names, but numbered differently
        filename[5] = '0' + i / 10;
        filename[6] = '0' + i % 10;
        if (!SD.exists(filename)) {                           //If a given filename doesn't exist, it's available
          datalog = SD.open(filename, FILE_WRITE);            //Create file with that name
          SDactive = true;                                    //Activate SD logging since file creation was successful
          Serial.println("Logging to: " + String(filename));  //Tell user which file contains the data for this run of the program
          break;                                              //Exit the for loop now that we have a file
        }
      }
      if (!SDactive) {
      Serial.println("No available file names; clear SD card to enable logging");
      } 
    }


  //Dallas Temp sensors setup
  if (!sensors.getAddress(inThermometer, 0)) {  //get address of first sensor; display error if not found
    Serial.println("Unable to find address for Device 0");
    if (SDactive) datalog.println("Unable to find address for Device 0");
  }
  if (!sensors.getAddress(outThermometer, 1)) { //repeat for second sensor
    Serial.println("Unable to find address for Device 1");
    if (SDactive) datalog.println("Unable to find address for Device 1");
  }
  sensors.setResolution(inThermometer, 9);  //set resolution for both sensors to 9 bits
  sensors.setResolution(outThermometer, 9);


  //IMU possible start up error message
  if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println("Failed to communicate with LSM9DS1.");  //Prints error message on startup if the IMU is not wired correctly.
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                   "work for an out of the box LSM9DS1 " \
                   "Breakout, but may need to be modified " \
                   "if the board jumpers are.");
  }


  delay(500);  //waits 500 ms


  //Prints header everytime on startup
  String headerLOG = "Mag(x), Mag(y), Mag(z), Temp1(C), Temp2(C), Pres(Atm), Time(ms)";  //Defines "headerLOG" as a string that contains the inscribed text.
  Serial.println(headerLOG);  //Prints the "headerLOG" to the serial monitor
  if (SDactive) {
    datalog.println(headerLOG);  //Prints the "headerLOG" to the SD-card
    datalog.close();
  }


  delay(500);  //waits 500 ms
  
}



void loop() {

  //Dallas Temp sensor loop
  sensors.requestTemperatures();  //Gets most recent temp data for all sensors
  inTemp = sensors.getTempC(inThermometer);  //Gets values from each sensor
  outTemp = sensors.getTempC(outThermometer);


  //IMU loop (gets the new data every loop and redefines the variables and then puts them in the string "magData"
  if ( imu.magAvailable() ){
    imu.readMag();
  }
  magX = imu.calcMag(imu.mx);
  magY = imu.calcMag(imu.my);
  magZ = imu.calcMag(imu.mz);
  magData = String(magX) + "," + String(magY) + "," + String(magZ);


  //Pressure Sensor loop
  pressureSensor1 = analogRead(Honeywell_1);                              //Reads the Honeywell Pressure Sensor's Analog Pin
  pressureSensorVoltage1 = pressureSensor1 * (5.0 / 1024);                //Converts the Digital Number to Voltage
  PressurePSI1 = (pressureSensorVoltage1 - (0.1 * 5.0)) / (4.0 / 15.0);   //Converts the Voltage to Pressure in PSI
  PressureATM1 = PressurePSI1 / 14.696;                                   //Converts Pressure from PSI to ATM


  //Creates the string "dataLOG" and includes the device measurements listed, which is then printed to the serial monitor, the xBee monitor, and the SD-card.
  String dataLOG = magData + "," + String(inTemp) + "," + String(outTemp) + "," + PressureATM1 + "," + millis()/1000;
    Serial.println(dataLOG);
    if (SDactive) {
      datalog = SD.open(filename, FILE_WRITE);
      datalog.println(dataLOG);                                
      datalog.close();   
    }


  delay(1000);  //waits 1 sec before re-running the loop
  
}
