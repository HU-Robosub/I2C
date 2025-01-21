#include <Arduino.h>
#include <Wire.h>
#include <WireSlave.h>
#include <WirePacker.h>
#include <ESP32Servo.h>
#include <WireSlaveRequest.h>

// I2C Config
#define sdaPin 10
#define sclPin 11
#define i2cAddr 0x08
#define i2cSlaveAddr 0x40
#define maxBytesLength 32

#define redLedPin 18
#define greenLedPin 19
#define blueLedPin 20

// Global Timing Variables
static unsigned long lastWireTransmit = 0;
static unsigned long lastWireRequest = 0;
static unsigned long lastHardwareUpdate = 0;

int receivedValue = 0;

struct actuator {
  int redLedValue;
  int greenLedValue;
  int blueLedValue;
} globalValue;


// Enum for identifying
enum idReceive {
  redPotId = 11,
  greenPotId = 12,
  BluePotId = 13,
  redLedId = 21,
  greenLedId = 22,
  blueLedId = 23
};


void setup() {
  Serial.begin(115200);

  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);

  // Initialize I2C sclPin    
  bool success = WireSlave.begin(sdaPin, sclPin, i2cAddr);
  if (!success) {
    Serial.println("I2C initialization failed!");
    while (1) delay(100);
  }

  Wire.setClock(100000);         // Set I2C clock to 400kHz

  WireSlave.onReceive(receiveEvent);
//  WireSlave.onRequest(requestEvent);

  Serial.println("I2C Initialized.");
  Serial.print("Address: ");
  Serial.println(i2cAddr, HEX);
  Serial.print("SDA Pin: ");
  Serial.println(sdaPin);
  Serial.print("SCL Pin: ");
  Serial.println(sclPin);
}


void loop() {
  // put your main code here, to run repeatedly:
  WireSlave.update();

  if (millis() - lastHardwareUpdate > 20) {
    updateHardware();

    lastHardwareUpdate = millis();
  }

  delay(10);
}

void updateHardware() {
  
  analogWrite(redLedPin, globalValue.redLedValue);
  analogWrite(greenLedPin, globalValue.greenLedValue);
  analogWrite(blueLedPin, globalValue.blueLedValue);

  // Print the values to the Serial Monitor
  Serial.print("Red LED Value: ");
  Serial.println(globalValue.redLedValue);

  Serial.print("Green LED Value: ");
  Serial.println(globalValue.greenLedValue);

  Serial.print("Blue LED Value: ");
  Serial.println(globalValue.blueLedValue);
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

  Serial.print("Received Value (Reconstructed): ");
  Serial.println(receivedValue);

  switch (id) {
    case redLedId: {
      int pwmValue = map(receivedValue, 0, 4095, 0, 255);
      globalValue.redLedValue = pwmValue;
      break;
    }
    
    case greenLedId:{
      int pwmValue = map(receivedValue, 0, 4095, 0, 255);
      globalValue.greenLedValue = pwmValue;
      break;
    }
    
    case blueLedId: {
      int pwmValue = map(receivedValue, 0, 4095, 0, 255);
      globalValue.blueLedValue = pwmValue;
      break;
    }
    
    default: {
      Serial.println("Error: Unknown identification byte.");
      break;
    }
  }
}
