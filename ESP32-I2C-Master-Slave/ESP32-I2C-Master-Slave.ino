#include <Arduino.h>
#include <Wire.h>
#include <WireSlave.h>
#include <WirePacker.h>
#include <ESP32Servo.h>
#include <WireSlaveRequest.h>

// I2C Config
#define sdaPin 10
#define sclPin 11
#define i2cAddr 0x40
#define i2cSlaveAddr 0x08
#define maxBytesLength 32

#define potRedPin 18
#define potGreenPin 19
#define potBluePin 20

//#define ledRedPin 18
//#define ledGreenPin 19
//#define ledBluePin 20

// Global Timing Variables
static unsigned long lastWireTransmit = 0;
static unsigned long lastWireRequest = 0;
static unsigned long lastHardwareUpdate = 0;

int receivedValue = 0;

struct sensor {
  int redPotValue;
  int greenPotValue;
  int bluePotValue;
};

//struct actuatorValue {
//  int redPotValue
//  int greenPotValue;
//  int bluePotValue;
//};

// Enum for identifying
enum idReceive {
  potRedId = 11,
  potGreenId = 12,
  potYellowId = 13,
  ledRedId = 21,
  ledGreenId = 22,
  ledYellowId = 23
};


void setup() {
  Serial.begin(115200);

  pinMode(potRedPin, INPUT);
  pinMode(potGreenPin, INPUT);
  pinMode(potBluePin, INPUT);

//  pinMode(ledRedPin, OUTPUT);
//  pinMode(ledGreenPin, OUTPUT);
//  pinMode(ledBluePin, OUTPUT);

  // Initialize I2C slavesclPin    
  bool success = WireSlave.begin(sdaPin, sclPin, i2cAddr);
  if (!success) {
    Serial.println("I2C slave initialization failed!");
    while (1) delay(100);
  }

  Wire.setClock(400000);         // Set I2C clock to 400kHz

//  WireSlave.onReceive(receiveEvent);
//  WireSlave.onRequest(requestEvent);

  Serial.println("I2C Initialized.");
  Serial.print("Slave Address: ");
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
    sendData(, int dataPacket);
    
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


void updateHardware() {
  struct sensor setValue;
  
  setValue.redPotValue = analogRead(potRedPin);
  setValue.greenPotValue = analogRead(potGreenPin);
  setValue.bluePotValue = analogRead(potBluePin);

}


void sendData(uint8_t id, int dataPacket) {
  // Pack the data
  WirePacker packer;
  packer.write(id);                // Identification byte
  packer.write((uint8_t)(dataPacket >> 8));  // High byte
  packer.write((uint8_t)(dataPacket & 0xFF)); // Low byte
  packer.end();

  // Send the data to the slave
  Wire.beginTransmission(i2cSlaveAddr);
  
  while (packer.available()) {
    Wire.write(packer.read());
  }
  byte transmissionStatus = Wire.endTransmission();

  // Handle transmission status
  if (transmissionStatus == 0) {
    Serial.println(" Transmission successful.");
  } else {
    Serial.print(" Transmission failed. Error code: ");
    Serial.println(transmissionStatus);
  }
}


int requestDataFromSlave() {

  // Create a WireSlaveRequest object
  WireSlaveRequest slaveReq(Wire, i2cSlaveAddr, maxBytesLength);

  slaveReq.setRetryDelay(5);

  // Attempt to read a packet from the slave
  bool success = slaveReq.request();

  if (success) {
    if (slaveReq.available() >= 2) {
      uint8_t highByte = slaveReq.read(); // Read high byte
      uint8_t lowByte = slaveReq.read();  // Read low byte
      int receivedValue = (highByte << 8) | lowByte; // Reconstruct value
  
      Serial.print("Received from Slave (Reconstructed Value): ");
      Serial.println(receivedValue);
  
      return receivedValue;
    } else {
      Serial.println("Not enough bytes received from Slave.");
      return -1; // Indicate error
    }
  } else {
    Serial.print("Request failed: ");
    Serial.println(slaveReq.lastStatusToString());
    return -1; // Indicate error
  }
}


void receiveEvent(int howMany) {
  Serial.print(howMany);

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

  switch (id) {
    case ledRedId: {
      int pwmValue = map(receivedValue, 0, 4095, 0, 255);
      analogWrite(ledRedPin, pwmValue);
      break;
    }
    case ledGreenId: {
      int pwmValue = map(receivedValue, 0, 4095, 0, 255);
      analogWrite(ledGreenPin, pwmValue);
      break;
    }
    case ledBlueId: {
      int pwmValue = map(receivedValue, 0, 4095, 0, 255);
      analogWrite(ledBluePin, pwmValue);
      break;
    }
    default: {
      Serial.println("Error: Unknown identification byte.");
      break;
    }
  }
}


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
