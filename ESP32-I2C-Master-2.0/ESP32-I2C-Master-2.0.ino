#include <Arduino.h>
#include <Wire.h>
#include <WireSlave.h>
#include <WirePacker.h>
#include <ESP32Servo.h>
#include <WireSlaveRequest.h>


#define sdaPin 10
#define sclPin 11
#define i2cAddr 0x40
#define i2cSlaveAddr 0x08
#define maxBytesLength 32


#define potRedPin 18
#define potGreenPin 19
#define potBluePin 20


#define redPotId 11
#define greenPotId 12
#define bluePotId 13

#define redLedId 21
#define greenLedId 22
#define blueLedId 23

bool master;

int receivedValue = 0;

// Global Timing Variables
static unsigned long lastWireTransmit = 0;
//static unsigned long lastWireRequest = 0;
static unsigned long lastHardwareUpdate = 0;

struct sensor {
  int redPotValue;
  int greenPotValue;
  int bluePotValue;
} globalValue;


void setup() {
  Serial.begin(115200);

  pinMode(potRedPin, INPUT);
  pinMode(potGreenPin, INPUT);
  pinMode(potBluePin, INPUT);

  // Initialize I2C sclPin    
  bool success = WireSlave.begin(sdaPin, sclPin, i2cAddr);
  if (!success) {
    Serial.println("I2C initialization failed!");
    while (1) delay(100);
  }

  Wire.setClock(100000);         // Set I2C clock to 400kHz

  WireSlave.onReceive(receiveEvent);
  
  Serial.println("I2C Initialized.");
  Serial.print("I2C Address: ");
  Serial.println(i2cAddr, HEX);
  Serial.print("SDA Pin: ");
  Serial.println(sdaPin);
  Serial.print("SCL Pin: ");
  Serial.println(sclPin);
}


void loop() {
  WireSlave.update();

  if (millis() - lastHardwareUpdate > 20) {
    updateHardware();

    lastHardwareUpdate = millis();
  }
  
    // Send potentiometer values to the slave
  if (millis() - lastWireTransmit > 50) {
    sendData(redLedId, globalValue.redPotValue);
    sendData(greenLedId, globalValue.greenPotValue);
    sendData(blueLedId, globalValue.bluePotValue);
    
    lastWireTransmit = millis();
  }

//  // Request data from the slave
//  if (millis() - lastWireRequest > 75) {
//    int receivedValue = requestDataFromSlave();
//
//    lastWireRequest = millis();
//  }

  delay(10); // Small delay for stability
}

// Function to switch to master mode
void switchToMaster() {
  WireSlave.end();
  
  bool success = Wire.begin(sdaPin, sclPin);
  if (!success) {
    Serial.println("I2C initialization failed!");
    while (1);
  }
}

// Function to switch back to slave mode
void switchToSlave() {
  Wire.end();
  
  bool success = WireSlave.begin(sdaPin, sclPin, i2cAddr);
  if (!success) {
    Serial.println("I2C initialization failed!");
    while (1);
  }
  
  WireSlave.onReceive(receiveEvent);
  WireSlave.onRequest(requestEvent);
}


void updateHardware() {
  
  globalValue.redPotValue = analogRead(potRedPin);
  globalValue.greenPotValue = analogRead(potGreenPin);
  globalValue.bluePotValue = analogRead(potBluePin);
}


void sendData(uint8_t id, int dataPacket) {
  switchToMaster();  // Switch to master mode
  
  WirePacker packer;
  packer.write(id);                // Identification byte
  packer.write((uint8_t)(dataPacket >> 8));  // High byte
  packer.write((uint8_t)(dataPacket & 0xFF)); // Low byte
  packer.end();

  Wire.beginTransmission(i2cSlaveAddr);
  
  while (packer.available()) {
    Wire.write(packer.read());
  }
  
  byte transmissionStatus = Wire.endTransmission();

  if (transmissionStatus == 0) {
    Serial.println(" Transmission successful.");
  } else {
    Serial.print(" Transmission failed. Error code: ");
    Serial.println(transmissionStatus);
  }
  
  switchToSlave();  // Return to slave mode
}


// Callback for data received from the master
void receiveEvent(int howMany) {
    if (howMany < 3) {
        Serial.println("Error: Not enough bytes received.");
        return;
    }

    // First byte is the identification byte
    uint8_t id = WireSlave.read();
    Serial.print("Identification Byte Received: ");
    Serial.println(id);

    // Following bytes are the high and low parts of the value
    int highByte = WireSlave.read();  // Read the high byte
    int lowByte = WireSlave.read();   // Read the low byte
    receivedValue = (highByte << 8) | lowByte; // Reconstruct the 16-bit value

    Serial.print("Received Value from Master (Reconstructed): ");
    Serial.println(receivedValue);
}


// Callback for request from the master
void requestEvent() {
    Serial.println("requestEvent called. Sending data to Master...");

    // Prepare the received value to send back
    byte highByte = (receivedValue >> 8) & 0xFF;  // Extract the high byte
    byte lowByte = receivedValue & 0xFF;          // Extract the low byte

    // Debug output for sent data
    Serial.print("Data sent to Master: High Byte = ");
    Serial.print(highByte);
    Serial.print(", Low Byte = ");
    Serial.println(lowByte);

    // Send the bytes back to the master
    WireSlave.write(highByte);
    WireSlave.write(lowByte);
}
