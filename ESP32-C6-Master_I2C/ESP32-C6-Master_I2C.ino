#include <Wire.h>

#define SLAVE_ADDR 0x08 // I2C address of the slave device

void setup() {
  Wire.begin(6, 7); // Initialize I2C as master
  Serial.begin(115200);
}

void loop() {
  Wire.beginTransmission(SLAVE_ADDR); // Start I2C transmission
  String message = "Hello from Master!";
  Wire.write(message.c_str()); // Send message to slave
  Wire.endTransmission(); // Stop I2C transmission

  Serial.println("Message sent to slave: " + message);

  delay(1000); // Wait 1 second before sending again
}
