//#include <Arduino.h>
//#include <Wire.h>
//#include <WirePacker.h>
//#include <WireSlaveRequest.h>
//
//#define SDA_PIN 10
//#define SCL_PIN 11
//#define I2C_SLAVE_ADDR 0x08
//
//#define LED 21
//
//#define POT_RED 13
//#define POT_GREEN 17
//#define POT_YELLOW 18
//
//// Set the maximum number of bytes the slave can send
//#define MAX_SLAVE_RESPONSE_LENGTH 32
//
//void setup() {
//    Serial.begin(115200);          // Start serial for debug output
//    Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C with SDA and SCL pins
//    Wire.setClock(100000);         // Set I2C clock to 100kHz
//
//    pinMode(LED, OUTPUT);          // Initialize LED as an output
//    pinMode(POT_RED, INPUT);           // Initialize POT as an input
//    pinMode(POT_GREEN, INPUT);           // Initialize POT as an input
//    pinMode(POT_YELLOW, INPUT);           // Initialize POT as an input
//
//    digitalWrite(LED, LOW);        // Ensure LED is OFF at startup
//
//    Serial.println("I2C Master Initialized.");
//}
//
//void loop() {
//    static unsigned long lastWireTransmit = 0;
//    static unsigned long lastWireRequest = 0;
//
//    // Send potentiometer value every 100ms
//    if (millis() - lastWireTransmit > 50) {
//        int potRedValue = analogRead(POT_RED);
//        Serial.print("Potentiometer Value: ");
//        Serial.println(potRedValue);
//
//        // Pack the data
//        WirePacker packer;
//        packer.write((uint8_t)(potRedValue >> 8));  // High byte
//        packer.write((uint8_t)(potRedValue & 0xFF)); // Low byte
//        packer.end();  // Close the packet
//
//        // Send the data to the slave
//        Wire.beginTransmission(I2C_SLAVE_ADDR);
//        while (packer.available()) {
//            Wire.write(packer.read());
//        }
//        byte transmissionStatus = Wire.endTransmission();
//
//        // Handle transmission status
//        if (transmissionStatus == 0) {
//            Serial.println("Transmission successful.");
//        } else {
//            Serial.print("Transmission failed. Error code: ");
//            Serial.println(transmissionStatus);
//        }
//
//        lastWireTransmit = millis();
//    }
//
//    // Request data from the slave every 75ms
//    if (millis() - lastWireRequest > 75) {
//        Serial.println("Requesting data from Slave...");
//
//        // Create a WireSlaveRequest object
//        WireSlaveRequest slaveReq(Wire, I2C_SLAVE_ADDR, MAX_SLAVE_RESPONSE_LENGTH);
//
//        // Optional: set retry delay between failed attempts (default: 10ms)
//        slaveReq.setRetryDelay(5);
//
//        // Attempt to read a packet from the slave
//        bool success = slaveReq.request();
//
//        if (success) {
//            // Check if enough data is available (2 bytes for your case)
//            if (slaveReq.available() >= 2) {
//                uint8_t highByte = slaveReq.read(); // Read high byte
//                uint8_t lowByte = slaveReq.read();  // Read low byte
//                int receivedValue = (highByte << 8) | lowByte; // Reconstruct value
//
//                Serial.print("Received from Slave (Reconstructed Value): ");
//                Serial.println(receivedValue);
//
//                // Control LED based on received value
//                if (receivedValue >= 2047) { // Adjust threshold if necessary
//                    digitalWrite(LED, HIGH);
//                    Serial.println("LED ON (Received Value >= 2047)");
//                } else {
//                    digitalWrite(LED, LOW);
//                    Serial.println("LED OFF (Received Value < 2047)");
//                }
//            } else {
//                Serial.println("Not enough bytes received from Slave.");
//            }
//        } else {
//            // If something went wrong, print the error status
//            Serial.print("Request failed: ");
//            Serial.println(slaveReq.lastStatusToString());
//        }
//
//        lastWireRequest = millis();
//    }
//
//    delay(10); // Avoid blocking
//}

#include <Arduino.h>
#include <Wire.h>
#include <WirePacker.h>
#include <WireSlaveRequest.h>

// I2C Config
#define SDA_PIN 10
#define SCL_PIN 11
#define I2C_SLAVE_ADDR 0x08
#define MAX_SLAVE_RESPONSE_LENGTH 32

// Hardware Pins
#define LED 21
#define POT_RED 4
#define POT_GREEN 17
#define POT_YELLOW 18

// Global Timing Variables
static unsigned long lastWireTransmit = 0;
static unsigned long lastWireRequest = 0;

// Threshold for LED Control
#define LED_THRESHOLD 2047

// Identification Bytes
#define ID_RED 1
#define ID_GREEN 2
#define ID_YELLOW 3

// Function Prototypes
void sendPotentiometerData(uint8_t potPin, uint8_t potID, const char* potName);
int requestDataFromSlave();

void setup() {
    Serial.begin(115200);          // Start serial for debug output
    Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C with SDA and SCL pins
    Wire.setClock(100000);         // Set I2C clock to 100kHz

    pinMode(LED, OUTPUT);          // Initialize LED as an output
    pinMode(POT_RED, INPUT);       // Initialize RED potentiometer as an input
    pinMode(POT_GREEN, INPUT);     // Initialize GREEN potentiometer as an input
    pinMode(POT_YELLOW, INPUT);    // Initialize YELLOW potentiometer as an input

    digitalWrite(LED, LOW);        // Ensure LED is OFF at startup

    Serial.println("I2C Master Initialized.");
}

void loop() {
    // Send potentiometer values to the slave
    if (millis() - lastWireTransmit > 50) {
        sendPotentiometerData(POT_RED, ID_RED, "RED");
        sendPotentiometerData(POT_GREEN, ID_GREEN, "GREEN");
        sendPotentiometerData(POT_YELLOW, ID_YELLOW, "YELLOW");
        lastWireTransmit = millis();
    }

    // Request data from the slave
    if (millis() - lastWireRequest > 75) {
        int receivedValue = requestDataFromSlave();

        // Control LED based on received value
        if (receivedValue >= LED_THRESHOLD) {
            digitalWrite(LED, HIGH);
            Serial.println("LED ON (Received Value >= Threshold)");
        } else {
            digitalWrite(LED, LOW);
            Serial.println("LED OFF (Received Value < Threshold)");
        }

        lastWireRequest = millis();
    }

    delay(10); // Avoid blocking
}

// Sends the data of a single potentiometer to the slave
void sendPotentiometerData(uint8_t potPin, uint8_t potID, const char* potName) {
    int potValue = analogRead(potPin);
    Serial.print(potName);
    Serial.print(" Potentiometer Value: ");
    Serial.println(potValue);

    // Pack the data
    WirePacker packer;
    packer.write(potID);                // Identification byte
    packer.write((uint8_t)(potValue >> 8));  // High byte
    packer.write((uint8_t)(potValue & 0xFF)); // Low byte
    packer.end();  // Close the packet

    // Send the data to the slave
    Wire.beginTransmission(I2C_SLAVE_ADDR);
    while (packer.available()) {
        Wire.write(packer.read());
    }
    byte transmissionStatus = Wire.endTransmission();

    // Handle transmission status
    if (transmissionStatus == 0) {
        Serial.print(potName);
        Serial.println(" Transmission successful.");
    } else {
        Serial.print(potName);
        Serial.print(" Transmission failed. Error code: ");
        Serial.println(transmissionStatus);
    }
}

// Requests data from the slave and returns the reconstructed value
int requestDataFromSlave() {
    Serial.println("Requesting data from Slave...");

    // Create a WireSlaveRequest object
    WireSlaveRequest slaveReq(Wire, I2C_SLAVE_ADDR, MAX_SLAVE_RESPONSE_LENGTH);

    // Optional: set retry delay between failed attempts (default: 10ms)
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
