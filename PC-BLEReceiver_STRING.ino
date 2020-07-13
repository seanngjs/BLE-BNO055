/*
  BLE Code Modified from SensorTag Button example in the ArduinoBLE library.
  Further BLE Code written by Sean Ng & Jordan Brown. March 2020.  


  This example scans for BLE peripherals until an IMUSensor peripheral is discovered.
  It then connects to it, discovers the attributes of the 0xffe0 service,
  subscribes to the IMUSensorData characteristic (UUID 0xffe1). 

  CONNECTION: This code is to be uploaded to the Receiver Arduino which is receiving data
  and connected to a computer. The Receiver Arduino is essentially used as a USB Bluetooth dongle.

  The hardware: 
  - Arduino Nano 33 IoT (connected to computer via USB)
  - USB Cable
  - Computer

  Libraries required:
  - ArduinoBLE

*/

#include <ArduinoBLE.h>

void setup() {
  Serial.begin(115200);
//  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  Serial.println("BLE Central - Receiver");
  Serial.println("Make sure to turn on the device.");

  // start scanning for peripheral
  BLE.scan();
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    // Check if the peripheral is a IMUSensor, the local name will be:
    // "IMUSensor"
    if (peripheral.localName() == "IMUSensor") {
      // stop scanning
      BLE.stopScan();

      monitorBLEperipheral(peripheral);

      // peripheral disconnected, start scanning again
      BLE.scan();
    }
  }
}

void monitorBLEperipheral(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");
  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering service 0xffe0 ...");
  if (peripheral.discoverService("180F")) {
    Serial.println("Service discovered");
  } else {
    Serial.println("Attribute discovery failed.");
    peripheral.disconnect();

    while (1);
    return;
  }

  // retrieve the IMUSensorData characteristic
  BLECharacteristic IMUSensorData = peripheral.characteristic("2A19");
//  BLEStringCharacteristic IMUSensorData = peripheral.characteristic("2A19", BLERead | BLEWrite, 512);
  
  // subscribe to the simple key characteristic
  Serial.println("Subscribing to IMUSensorData characteristic ...");
  if (!IMUSensorData) {
    Serial.println("no IMUSensorData characteristic found!");
    peripheral.disconnect();
    return;
  } 
    else if (!IMUSensorData.canSubscribe()) {
    Serial.println("IMUSensorData characteristic is not subscribable!");
    peripheral.disconnect();
    return;
  } 
    else if (!IMUSensorData.subscribe()) {
    Serial.println("subscription failed!");
    peripheral.disconnect();
    return;
  } 
    else {
    Serial.println("Subscribed to IMUSensorData characteristic");
//    Serial.println("Press the right and left buttons on your SensorTag.");
  }

  while (peripheral.connected()) {
    // while the peripheral is connected

    // check if the value of the simple key characteristic has been updated
    if (IMUSensorData.valueUpdated()) {
//    if (IMUSensorData.written()){
      // yes, get the value, characteristic is 1 byte so use byte value
//      long value = 0 ;
//      int ReceivingArray[2] = {0,0}; 

// THIS SECTION CONVERTS THE RECEIVED CHARACTERISTIC FROM UNSIGNED CHAR TO STRING // 
      String str;
      int length = IMUSensorData.valueLength();
      const uint8_t* val = IMUSensorData.value();
      str.reserve(length);

      for (int i = 0; i<length; i++){
        str += (char)val[i];
      }

      Serial.println(str);
        
//      IMUSensorData.readValue(value);

      
//      Serial.println(value);
//      IMUSensorData.readValue(ReceivingArray,2);
//      Serial.println(ReceivingArray[0]);
//      Serial.println(ReceivingArray[1]);
//      for(int i = 0; i < 1; i++)
//      {
//        Serial.println(ReceivingArray[i]);
//      }

//      if (value & 0x01) {
//        // first bit corresponds to the right button
//        Serial.println("Right button pressed");
//      }
//
//      if (value & 0x02) {
//        // second bit corresponds to the left button
//        Serial.println("Left button pressed");
      }
    }
  

  Serial.println("BLE disconnected!");
}
