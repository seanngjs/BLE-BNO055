/* IMU Code Written by Sean Ng. Modified from Adafruit sensorapi. March 2019.
   available from https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code 
 
  BLE Code Modified from BatteryMonitor example in the ArduinoBLE library.
  Further BLE Code written by Sean Ng & Jordan Brown. March 2020.
  

  CONNECTION: This code is to be uploaded to the Sender Arduino which is transmitting data. 
  The circuit:
  - Arduino Nano 33 IoT
  - Adafruit BNO055 IMU Sensor

  Libraries required:
  - ArduinoBLE
  - Wire
  - Adafruit_Sensor
  - Adafruit_BNO055
  
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
unsigned long previousTime=0;           // end time of the last period in microseconds
unsigned long currentTime=0;
unsigned long deltaTime=0;
const unsigned long period=10000;       // time period for pulse counting in microseconds
unsigned long last = 0;
unsigned long time;
volatile unsigned long cont=0;

/* BLE prevoid
*/
#include <ArduinoBLE.h>

 // BLE IMU Service
BLEService IMUService("180F");

// BLE Orientation Characteristic
//BLECharacteristic IMUSensorData("2A19",  // standard 16-bit characteristic UUID
  //  BLERead | BLENotify, 2); // remote clients will be able to get notifications if this characteristic changes
BLEStringCharacteristic IMUSensorData("2A19", BLERead | BLENotify, 512);

// Note: BLEStringCharacteristic was not mentioned in the ArduinoBLE website but exists in the library


/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.

   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground

*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
//  Serial.print("\t");
//  if (!system)
//  {
//    Serial.print("! ");
//  }

  /* Display the individual values */
//  Serial.print("Sys:");
  Serial.print(system, DEC);
//  Serial.print(" G:");
  Serial.print(gyro, DEC);
//  Serial.print(" A:");
  Serial.print(accel, DEC);
//  Serial.print(" M:");
  Serial.print(mag, DEC);
 

}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  pinMode(13, OUTPUT);
//  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  // begin BLE initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }
  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("IMUSensor");
  BLE.setAdvertisedService(IMUService); // add the service UUID
  IMUService.addCharacteristic(IMUSensorData); // add the IMU characteristic
  BLE.addService(IMUService); // Add the IMU service
  IMUSensorData.writeValue("hello"); // set initial value for this characteristic
  
  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");  
  String address = BLE.address();
  Serial.print("Local address is: ");
  Serial.println(address);


  delay(1000);

  /* Display some basic information on this sensor */
//  displaySensorDetails();

  /* Optional: Display current status */
//  displaySensorStatus();

  bno.setExtCrystalUse(true);
  Serial.flush();
}


/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete
*/
/**************************************************************************/
void loop(void)
{
    // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the IMU every period
    // while the central is connected:
    while (central.connected()) {
      currentTime = micros();
      deltaTime = currentTime-previousTime;

        /* Check if the time in between samples equals sampling period before taking a new sample */
      if (deltaTime >= period) 
        {
        /* Display the current time in microseconds */
        Serial.print(currentTime);
        Serial.print(",");
        previousTime = currentTime;
        /* Get a new sensor event */
        sensors_event_t event;
        bno.getEvent(&event);

        /* Calibration code */
        uint8_t system, gyro, accel, mag;
        system = gyro = accel = mag = 0;
        bno.getCalibration(&system, &gyro, &accel, &mag);
      
        /* The data should be ignored until the system calibration is > 0 */
      //  Serial.print("\t");
      //  if (!system)
      //  {
      //    Serial.print("! ");
      //  }
      
        /* Display the individual values */
      //  Serial.print("Sys:");
        Serial.print(system, DEC);
        int CalSys = system;
      //  Serial.print(" G:");
        Serial.print(gyro, DEC);
        int CalGy = gyro;
      //  Serial.print(" A:");
        Serial.print(accel, DEC);
        int CalAcc = accel;
      //  Serial.print(" M:");
        Serial.print(mag, DEC);
        int CalMag = mag;


        
        /* Display the floating point data */
        imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//        int Ox = event.orientation.x * 10000;
//        int Oy = event.orientation.y * 10000;
        float Ox = event.orientation.x;
        float Oy = event.orientation.y;
        float Oz = event.orientation.z;
        float LAx = linearaccel.x();
        float LAy = linearaccel.y();
        float LAz = linearaccel.z();
        
        
        // Stores each IMU data type in a string 'buff' // 
        String buff;
        buff += String(currentTime);
        buff += F(",");
        buff += String(Ox,2);
        buff += F(",");
        buff += String(Oy,2);
        buff += F(",");
        buff += String(Oz,2);
        buff += F(",");
        buff += String(LAx,2);
        buff += F(",");
        buff += String(LAy,2);
        buff += F(",");
        buff += String(LAz,2);
        buff += F(",");
        buff += String(deltaTime);
        buff += F(",");
        buff += String(cont);
        buff += F(",");
        buff += String(CalSys);
        buff += String(CalGy);
        buff += String(CalAcc);
        buff += String(CalMag);
        // Sends buff using the BLE connection
        IMUSensorData.writeValue(buff);

 
        // Prints each IMU data type in the serial monitor // 
        Serial.print(Ox);
        Serial.print(",");
        Serial.print(Oy);
        Serial.print(",");
        Serial.print(Oz);
        Serial.print(",");
        Serial.print(linearaccel.x());
        Serial.print(",");
        Serial.print(linearaccel.y());
        Serial.print(",");
        Serial.print(linearaccel.z());
        Serial.print(",");
        Serial.print(deltaTime);
        Serial.print(",");
        cont++;
        Serial.print(cont);
        Serial.print(",");
        displayCalStatus();
//        /* New line for the next sample */
        Serial.println("");
//        
    
      }
    
            
          }
        }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }




  
  /* Optional: Display calibration status */
//  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();
