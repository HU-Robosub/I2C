//// WireSlave Receiver
//// by Gutierrez PS <https://github.com/gutierrezps>
//// ESP32 I2C slave library: <https://github.com/gutierrezps/ESP32_I2C_Slave>
//// based on the example by Nicholas Zambetti <http://www.zambetti.com>
//
//// Demonstrates use of the WireSlave library for ESP32.
//// Receives data as an I2C/TWI slave device; data must
//// be packed using WirePacker.

#include <Arduino.h>
#include <Wire.h>
#include <WireSlave.h>

#define sdaPin 10
#define sclPin 11
#define i2cSlaveAddr 0x08

#define redLedPin 18
#define greenLedPin 19


int receivedValue = 0;
                                              

void setup() {
  Serial.begin(115200); // Use a high baud rate

  // Initialiseer I2C-slave
  bool success = WireSlave.begin(sdaPin, sclPin, i2cSlaveAddr);
  if (!success) {
    Serial.println("I2C slave init failed");
    while (1) delay(100);
  }

  WireSlave.onReceive(receiveEvent); // Register the onReceive handler
  WireSlave.onRequest(requestEvent); // Register the onRequest handler
  Serial.println("Data I2C initialized");

  // Configure control pins as outputs for AD0 control
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  
  Serial.println("Setup Done");
}


// Updated receiveEvent function
void receiveEvent(int howMany) {
  
  if (howMany > 0) {
    
  receivedValue = WireSlave.read(); // Read the first byte as a command
  analogWrite(redLedPin, receivedValue);
  }
}

// Function to handle master requests for roll pitch
void requestEvent() {

}


void loop() {
  WireSlave.update(); // Keep the I2C slave active

  // Perform other tasks or simply allow interrupts to work
  delay(1); // Short delay for other processes and interrupts
}
