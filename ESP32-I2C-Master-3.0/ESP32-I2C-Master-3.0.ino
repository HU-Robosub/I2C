// Wire Master Writer
// by Gutierrez PS <https://github.com/gutierrezps>
// ESP32 I2C slave library: <https://github.com/gutierrezps/ESP32_I2C_Slave>
// based on the example by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire and WirePacker libraries.
// Writes data to an ESP32 I2C/TWI slave device that
// uses ESP32 I2C Slave library.

#include <Arduino.h>
#include <Wire.h>
#include <WirePacker.h>
#include <WireSlaveRequest.h>
#include <U8g2lib.h>

// Primary I2C bus pins for slave communication
#define sdaPin 10
#define sclPin 11
#define i2cSlaveAddr 0x08
#define maxSlaveResponseLength 32

// Secondary I2C bus pins for OLED display
#define hardwareSdaPin 41
#define hardwareSclPin 42

#define rightPotPin 6
#define middPotPin 5
#define leftPotPin 4
#define touchButtonPin 8


static unsigned long lastWireTransmit = 0;
static unsigned long lastDisplayUpdate = 0;
static unsigned long lastWireRequest = 0;

struct PotentiometerData {
  uint16_t potLeft;
  uint16_t potMidd;
  uint16_t potRight;
};

PotentiometerData potData = {0, 0, 0};

struct MPUData {
    float roll, pitch;
};
MPUData mpuData[3]; // Array to store data for 3 MPU6050 sensors

// Initialize static labels only once
static bool labelsDrawn = false;

const int modesNum = 2;  // Number of modes
enum SystemMode {
    RAW_POT_TO_SERVO,
    SET_POT_TO_SERVO,
};
SystemMode currentMode = RAW_POT_TO_SERVO;

static bool lastTouchState = HIGH;  // Track the last state of the touch button


// Define the secondary I2C instance for slave communication
TwoWire communicationWire = TwoWire(1); 

// Create a WireSlaveRequest object
WireSlaveRequest slaveReq(communicationWire, i2cSlaveAddr, maxSlaveResponseLength);

// U8g2 object for OLED display on the primary I2C bus
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R1, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ hardwareSclPin, /* data=*/ hardwareSdaPin);


void setup() {
  Serial.begin(115200);

  // Initialize primary I2C bus for slave communication
  communicationWire.begin(sdaPin, sclPin);
  communicationWire.setClock(400000);

  // Initialize the secondary I2C bus for OLED display
  Wire.begin(hardwareSdaPin, hardwareSclPin);
  Wire.setClock(400000);

  // Initialize OLED
  u8g2.begin();
  
  // Display initialization message
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(0, 20, "Display Ready!");
  u8g2.sendBuffer();

  // Configure potentiometer pins
  pinMode(rightPotPin, INPUT);
  pinMode(middPotPin, INPUT);
  pinMode(leftPotPin, INPUT);
  pinMode(touchButtonPin, INPUT);

  scanI2CBuses();

  Serial.println("Setup complete.");
}

void scanI2CBuses() {
  Serial.println("Scanning primary I2C bus (communicationWire)...");
  for (uint8_t address = 1; address < 127; address++) {
    communicationWire.beginTransmission(address);
    if (communicationWire.endTransmission() == 0) {
      Serial.print("Found I2C device at address 0x");
      Serial.println(address, HEX);
    }
  }

  Serial.println("Scanning secondary I2C bus (Wire)...");
  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found I2C device at address 0x");
      Serial.println(address, HEX);
    }
  }
}

void updateOLED() {
  if (!labelsDrawn) {
    // Initialize the display once with labels
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_5x8_tr); // Compact font for values

    // Draw labels for the potentiometer and MPU values
    u8g2.drawStr(0, 10, "PotL:");
    u8g2.drawStr(0, 20, "PotM:");
    u8g2.drawStr(0, 30, "PotR:");
    u8g2.drawStr(0, 40, "MPU1R:");
    u8g2.drawStr(0, 50, "MPU1P:");
    u8g2.drawStr(0, 60, "MPU2R:");
    u8g2.drawStr(0, 70, "MPU2P:");
    u8g2.drawStr(0, 80, "MPU3R:");
    u8g2.drawStr(0, 90, "MPU3P:");

    labelsDrawn = true;
    
    u8g2.sendBuffer(); // Send the initial buffer to display the labels
  }

  u8g2.setFont(u8g2_font_5x8_tr);  // Use the compact font for numbers

  // Clear the previous values area (clear region where values are displayed)
  u8g2.setDrawColor(0);  // Set color to clear (background)
  u8g2.drawBox(40, 0, 58, 100); // Clear the value area (X=70, width=58, height=100)
  u8g2.setDrawColor(1);  // Restore the draw color (foreground)

  // Draw updated potentiometer values
  u8g2.setCursor(40, 10);  // Set the cursor position for Pot Left
  u8g2.print(potData.potLeft);

  u8g2.setCursor(40, 20);  // Set the cursor position for Pot Middle
  u8g2.print(potData.potMidd);

  u8g2.setCursor(40, 30);  // Set the cursor position for Pot Right
  u8g2.print(potData.potRight);

  // Draw updated MPU values (Roll and Pitch for each MPU)
  for (int i = 0; i < 3; ++i) {
    u8g2.setCursor(40, 40 + (i * 20));  // Adjust cursor for each MPU's roll
    u8g2.print(mpuData[i].roll);  // Display Roll

    u8g2.setCursor(40, 50 + (i * 20));  // Adjust cursor for each MPU's pitch
    u8g2.print(mpuData[i].pitch);  // Display Pitch
  }

  // Send the buffer to display the updated data
  u8g2.sendBuffer();
}

void sendPotDataToSlave() {
  potData.potRight = analogRead(rightPotPin);
  potData.potMidd = analogRead(middPotPin);
  potData.potLeft = analogRead(leftPotPin);

  Serial.print("Sending -> PotRight: ");
  Serial.print(potData.potRight);
  Serial.print(", PotMiddle: ");
  Serial.print(potData.potMidd);
  Serial.print(", PotLeft: ");
  Serial.println(potData.potLeft);

  WirePacker packer;
  packer.write((uint8_t)0x02); // Send pot data command
  packer.write((uint8_t*)&potData, sizeof(potData));
  packer.end();

  communicationWire.beginTransmission(i2cSlaveAddr);
  while (packer.available()) {
    communicationWire.write(packer.read());
  }
  
  communicationWire.endTransmission();
}

// Requests data from the slave and returns the reconstructed MPUData for 3 sensors
bool requestDataFromSlave() {
  Serial.println("Requesting data from Slave...");
  
  // Optional: set retry delay between failed attempts (default: 10ms)
  slaveReq.setRetryDelay(2);
  
  // Attempt to read a packet from the slave
  bool success = slaveReq.request();
  
  if (success) {
    if (slaveReq.available() >= 24) { // 2 bytes per sensor (roll + pitch) * 3 sensors * 2 bytes per float (for 4 bytes total)
      for (int i = 0; i < 3; ++i) {
        // Read 4 bytes for roll (float)
        uint8_t rollBytes[4];
        for (int j = 0; j < 4; ++j) {
          rollBytes[j] = slaveReq.read();
        }
        mpuData[i].roll = *((float*)rollBytes); // Convert bytes to float

        // Read 4 bytes for pitch (float)
        uint8_t pitchBytes[4];
        for (int j = 0; j < 4; ++j) {
          pitchBytes[j] = slaveReq.read();
        }
        mpuData[i].pitch = *((float*)pitchBytes); // Convert bytes to float

        // Optionally print each sensor's roll and pitch values
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(" Roll: ");
        Serial.print(mpuData[i].roll);
        Serial.print(", Pitch: ");
        Serial.println(mpuData[i].pitch);
      }
    return true;
    } else {
      Serial.println("Not enough bytes received from Slave.");
      return false; // Indicate error
      }
  } else {
    Serial.print("Request failed: ");
    Serial.println(slaveReq.lastStatusToString());
    return false; // Indicate error
  }
}

void sendModeChangeToSlave() {
  WirePacker packer;
  packer.write((uint8_t)0x01); // Mode-change command
  packer.write((uint8_t)currentMode); // Send the new mode (0: raw, 1: servo)
  packer.end();

  communicationWire.beginTransmission(i2cSlaveAddr);
  while (packer.available()) {
    communicationWire.write(packer.read());
  }
  communicationWire.endTransmission();

  Serial.println(currentMode ? "Mode switched to set Servo" : "Mode switched to Raw");
}


void loop() {
  int touchValue = digitalRead(touchButtonPin);  // Read the current touch button state

  // Check for a button press (state change from HIGH to LOW)
  if (touchValue == LOW && lastTouchState == HIGH) {
    currentMode = static_cast<SystemMode>((currentMode + 1) % modesNum);  // Toggle mode
    Serial.print("Current Mode: ");
    Serial.println(currentMode == RAW_POT_TO_SERVO ? "RAW_POT_TO_SERVO" : "SET_POT_TO_SERVO");
    sendModeChangeToSlave();
  }

  lastTouchState = touchValue;  // Update the last state

  // 1. Transmit potentiometer data to the slave every 20ms
  if (millis() - lastWireTransmit > 20) {
    sendPotDataToSlave();
    lastWireTransmit = millis();
  }

  // 2. Update the OLED every 100ms
  if (millis() - lastDisplayUpdate > 100) {
    potData.potRight = analogRead(rightPotPin);  // Update display with current pot values
    potData.potMidd = analogRead(middPotPin);
    potData.potLeft = analogRead(leftPotPin);

    updateOLED();
    lastDisplayUpdate = millis();
  }

  // Request data from the slave
  if (millis() - lastWireRequest > 75) {
    requestDataFromSlave();
    
    lastWireRequest = millis();
  }
}
