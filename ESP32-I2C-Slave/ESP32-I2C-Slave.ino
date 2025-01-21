//#include <Arduino.h>
//#include <WireSlave.h>
//#include <ESP32Servo.h>
//
//#define SDA_PIN 10
//#define SCL_PIN 11
//#define I2C_SLAVE_ADDR 0x08
//
//#define LED_PIN 21
//#define SERVO_PIN_180 13
//#define SERVO_PIN_360_LEFT 17
//#define SERVO_PIN_360_RIGHT 18
//
//int receivedValue = 0;
//Servo myServo;
//
//void receiveEvent(int howMany);
//void requestEvent();
//
//void setup() {
//    Serial.begin(115200);
//
//    // Initialize I2C slave
//    bool success = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR);
//    if (!success) {
//        Serial.println("I2C slave initialization failed!");
//        while (1) delay(100);
//    }
//
//    WireSlave.onReceive(receiveEvent);
//    WireSlave.onRequest(requestEvent);
//
//    // Initialize peripherals
//    pinMode(LED_PIN, OUTPUT);
//    myServo.attach(SERVO_PIN);
//
//    // Debug output
//    Serial.println("I2C Slave Initialized.");
//    Serial.print("Slave Address: ");
//    Serial.println(I2C_SLAVE_ADDR, HEX);
//    Serial.print("SDA Pin: ");
//    Serial.println(SDA_PIN);
//    Serial.print("SCL Pin: ");
//    Serial.println(SCL_PIN);
//}
//
//void loop() {
//    // Update I2C slave (essential to avoid missing events)
//    WireSlave.update();
//
//    // Control the LED based on the received value
//    if (receivedValue <= 2047) {
//        digitalWrite(LED_PIN, HIGH);
//        Serial.println("LED ON (Received Value <= 511)");
//    } else {
//        digitalWrite(LED_PIN, LOW);
//        Serial.println("LED OFF (Received Value > 511)");
//    }
//
//    // Map the received potentiometer value (0–1023) to the servo range (0–180 degrees)
//    int servoAngle = map(receivedValue, 0, 4095, 0, 180);
//    myServo.write(servoAngle); // Set the servo to the mapped angle
//
//    // Debug output to monitor the servo angle
//    Serial.print("Servo Angle: ");
//    Serial.println(servoAngle);
//
//    delay(10); // Small delay for stability
//}
//
//// Function called whenever data is received from the master
//void receiveEvent(int howMany) {
//    Serial.print("receiveEvent called. Bytes received: ");
//    Serial.println(howMany);
//
//    if (howMany == 2) { // Expecting 2 bytes
//        int highByte = WireSlave.read();  // Read the high byte
//        int lowByte = WireSlave.read();   // Read the low byte
//        receivedValue = (highByte << 8) | lowByte; // Reconstruct the 10-bit value
//
//        Serial.print("Received from Master (Reconstructed Value): ");
//        Serial.println(receivedValue);
//    } else {
//        Serial.print("Error: Expected 2 bytes but received ");
//        Serial.println(howMany);
//    }
//}
//
//// Function called whenever the master requests data
//void requestEvent() {
//    Serial.println("requestEvent called. Sending data to Master...");
//
//    // Prepare the received value to send back
//    byte highByte = (receivedValue >> 8) & 0xFF;  // Extract the high byte
//    byte lowByte = receivedValue & 0xFF;          // Extract the low byte
//
//    // Debug output for sent data
//    Serial.print("Data sent to Master: High Byte = ");
//    Serial.print(highByte);
//    Serial.print(", Low Byte = ");
//    Serial.println(lowByte);
//
//    // Send the bytes back to the master
//    WireSlave.write(highByte);
//    WireSlave.write(lowByte);
//}

#include <Arduino.h>
#include <WireSlave.h>
#include <ESP32Servo.h>

// I2C Config
#define SDA_PIN 10
#define SCL_PIN 11
#define I2C_SLAVE_ADDR 0x08

// Peripherals
#define LED_PIN 21
#define SERVO_PIN_180 13
#define SERVO_PIN_360_LEFT 17
#define SERVO_PIN_360_RIGHT 18

int receivedValue = 0;

// Servo instances for each servo motor
Servo servo180;
Servo servo360Left;
Servo servo360Right;

// Enum for identifying potentiometers
enum PotentiometerID {
    POT_RED = 1,   // Identification byte for RED potentiometer
    POT_GREEN = 2, // Identification byte for GREEN potentiometer
    POT_YELLOW = 3 // Identification byte for YELLOW potentiometer
};

void receiveEvent(int howMany);
void requestEvent();

void setup() {
    Serial.begin(115200);

    // Initialize I2C slave
    bool success = WireSlave.begin(SDA_PIN, SCL_PIN, I2C_SLAVE_ADDR);
    if (!success) {
        Serial.println("I2C slave initialization failed!");
        while (1) delay(100);
    }

    WireSlave.onReceive(receiveEvent);
    WireSlave.onRequest(requestEvent);

    // Initialize peripherals
    pinMode(LED_PIN, OUTPUT);
    servo180.attach(SERVO_PIN_180);
    servo360Left.attach(SERVO_PIN_360_LEFT);
    servo360Right.attach(SERVO_PIN_360_RIGHT);

    // Debug output
    Serial.println("I2C Slave Initialized.");
    Serial.print("Slave Address: ");
    Serial.println(I2C_SLAVE_ADDR, HEX);
    Serial.print("SDA Pin: ");
    Serial.println(SDA_PIN);
    Serial.print("SCL Pin: ");
    Serial.println(SCL_PIN);
}

void loop() {
    // Update I2C slave (essential to avoid missing events)
    WireSlave.update();

    delay(10); // Small delay for stability
}

// Function called whenever data is received from the master
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

    // Map the received potentiometer value (0–4095) to appropriate ranges
    switch (id) {
        case POT_RED: { // Controls the 180° servo
            int servoAngle = map(receivedValue, 0, 4095, 0, 180); // Map to 0–180°
            servo180.write(servoAngle);
            Serial.print("RED Potentiometer: Setting 180° servo to ");
            Serial.println(servoAngle);
            break;
        }

        case POT_GREEN: { // Controls the left 360° servo
            int servoSpeed = map(receivedValue, 0, 4095, 0, 180); // Map to speed range (0–180)
            servo360Left.write(servoSpeed);
            Serial.print("GREEN Potentiometer: Setting Left 360° servo to speed ");
            Serial.println(servoSpeed);
            break;
        }

        case POT_YELLOW: { // Controls the right 360° servo
            int servoSpeed = map(receivedValue, 0, 4095, 0, 180); // Map to speed range (0–180)
            servo360Right.write(servoSpeed);
            Serial.print("YELLOW Potentiometer: Setting Right 360° servo to speed ");
            Serial.println(servoSpeed);
            break;
        }

        default: {
            Serial.println("Error: Unknown identification byte.");
            break;
        }
    }
}

// Function called whenever the master requests data
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
