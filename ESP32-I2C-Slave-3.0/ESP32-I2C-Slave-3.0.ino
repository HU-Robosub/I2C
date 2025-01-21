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
#include <ESP32Servo.h>
#include <MPU6050.h>

#define sdaPin 10
#define sclPin 11
#define i2cSlaveAddr 0x08

// Define SDA and SCL pins for the secondary I2C bus
#define hardwareSdaPin 41
#define hardwareSclPin 42

#define servoLeftPin 16
#define servoMiddPin 17
#define servoRightPin 18

// Define GPIO pins for controlling the AD0 pins of each MPU6050
#define mpuLeftPin 35
#define mpuMiddPin 36
#define mpuRightPin 37


// Enum for identifying potentiometers
enum receiveId {
    modeSelect = 0x01,   // Identification byte for modeSelect
    potentiometerData = 0x02, // Identification byte for potentiometerData
};                                                    

// Struct voor potentiometer values
struct PotentiometerData {
  uint16_t potLeft;
  uint16_t potMidd;
  uint16_t potRight;
};
PotentiometerData receivedData; // Struct to receive data

struct MPUData {
  int16_t ax, ay, az;
};
MPUData mpuData[3]; // Array to store data for 3 MPU6050 sensors

struct rollPitchData {
  float roll, pitch;
};
rollPitchData RPData[3];

unsigned long previousMPUTime = 0; // To store the last time MPU data was updated
const unsigned long mpuInterval = 50; // Time interval (in ms) between MPU updates
static unsigned long previousStabilizeTime = 0; // Track the last stabilization time
unsigned long stabilizationInterval = 75;       // Stabilization interval in milliseconds (adjust as needed)

enum SystemMode {
    RAW_POT_TO_SERVO,
    SET_POT_TO_SERVO,
};
SystemMode currentMode = RAW_POT_TO_SERVO;

const float kp = 0.8;  // Proportional gain
const float ki = 0.15;  // Integral gain
const float kd = 0.02; // Derivative gain
const float alpha = 0.6; // Low-pass filter constant

// Limits for integral windup
const float maxIntegralError = 300.0;

// Variables for storing errors
float previousError[3] = {0.0, 0.0, 0.0};
float integralError[3] = {0.0, 0.0, 0.0};
float smoothedAy[3] = {0.0, 0.0, 0.0}; // Low-pass filtered acceleration values


// Create a TwoWire instance for the secondary I2C connection
TwoWire hardwareI2c = TwoWire(1);

MPU6050 mpu(MPU6050_DEFAULT_ADDRESS, &hardwareI2c); // Pass the I2C address and the hardwareI2c instance

Servo servoLeft, servoMidd, servoRight; // Servo-objects


void setup() {
  Serial.begin(250000); // Use a high baud rate

  // Initialiseer I2C-slave
  bool success = WireSlave.begin(sdaPin, sclPin, i2cSlaveAddr);
  if (!success) {
    Serial.println("I2C slave init failed");
    while (1) delay(100);
  }

  WireSlave.onReceive(receiveEvent); // Register the onReceive handler
  WireSlave.onRequest(requestEvent); // Register the onRequest handler
  Serial.println("Data I2C initialized");

  // Initialize the secondary I2C connection
  if (!hardwareI2c.begin(hardwareSdaPin, hardwareSclPin, 400000)) {
    Serial.println("Hardware I2C init failed");
    while (1) delay(100);
  }
  Serial.println("Hardware I2C initialized");

  // Initialiseer servo's
  servoLeft.attach(servoLeftPin);
  servoMidd.attach(servoMiddPin);
  servoRight.attach(servoRightPin);

  // Configure control pins as outputs for AD0 control
  pinMode(mpuLeftPin, OUTPUT);
  pinMode(mpuMiddPin, OUTPUT);
  pinMode(mpuRightPin, OUTPUT);

  // Ensure all AD0 pins are HIGH at the start
  digitalWrite(mpuLeftPin, HIGH);
  digitalWrite(mpuMiddPin, HIGH);
  digitalWrite(mpuRightPin, HIGH);

  // Initialize each MPU6050
  for (int i = 1; i <= 3; i++) {
    selectMPU(i);  // Activate the current MPU6050
    mpu.initialize();  // Initialize the MPU6050
    if (mpu.testConnection()) {
      Serial.print("MPU");
      Serial.print(i);
      Serial.println(" connected!");
    } else {
      Serial.print("MPU");
      Serial.print(i);
      Serial.println(" connection failed!");
    }
  }
  
  Serial.println("Setup Done");
}


// Updated receiveEvent function
void receiveEvent(int howMany) {
  
  if (howMany > 0) {
    uint8_t receiveId = WireSlave.read(); // Read the first byte as a command
    
    switch (receiveId) {
      case modeSelect: {
        handleModeSwitch(howMany);
        break;
      }
      case potentiometerData: {
        handlePotentiometerData(howMany);
        break;
      }
      default: {
        break;
      }
    }
  }
}

// Function to handle mode switching
void handleModeSwitch(int howMany) {
  
  if (howMany >= 1) {
    currentMode = static_cast<SystemMode>(WireSlave.read()); // Read the new mode
    
    previousError[0] = 0.0;
    previousError[1] = 0.0;
    previousError[2] = 0.0;
    
    integralError[0] = 0.0;
    integralError[1] = 0.0;
    integralError[2] = 0.0;

    // Set the initial smoothedAy values based on the potentiometer data
    smoothedAy[0] = map(receivedData.potLeft, 0, 4095, -16000, 16000);
    smoothedAy[1] = map(receivedData.potMidd, 0, 4095, -16000, 16000);
    smoothedAy[2] = map(receivedData.potRight, 0, 4095, -16000, 16000);
    
    Serial.print("Mode switched to: ");
    Serial.println(currentMode == RAW_POT_TO_SERVO ? "RAW_POT_TO_SERVO" : "SET_POT_TO_SERVO");
  }
}

// Updated function to handle potentiometer data
void handlePotentiometerData(int howMany) {
  
  if (howMany == sizeof(receivedData) + 1) {
    WireSlave.readBytes((char*)&receivedData, sizeof(receivedData));

    // Debug: Print received potentiometer values
    Serial.print("Received PotLeft: ");
    Serial.print(receivedData.potLeft);
    Serial.print(" PotMidd: ");
    Serial.print(receivedData.potMidd);
    Serial.print(" PotRight: ");
    Serial.println(receivedData.potRight);

    if (currentMode == RAW_POT_TO_SERVO) {
      // Direct servo control mode
      positionServos();
    } else if (currentMode == SET_POT_TO_SERVO) {
      // Just save the potentiometer data but do NOT directly update the servos
      Serial.println("Potentiometer data saved for SET_POT_TO_SERVO mode.");
    }
  }
}

void positionServos() {
  
  servoLeft.write(map(receivedData.potLeft, 0, 4095, 0, 180));
  servoMidd.write(map(receivedData.potMidd, 0, 4095, 0, 180));
  servoRight.write(map(receivedData.potRight, 0, 4095, 0, 180));
}

// Function to stabilize the servos based on `ax` values from MPU6050
void stabilizeServos() {
  
  for (int i = 0; i < 3; i++) {
    // Retrieve the `ay` value for the current MPU
    float currentAy = mpuData[i].ay; // Assume mpuData[i].ay contains the raw ax value
    smoothedAy[i] = alpha * smoothedAy[i] + (1 - alpha) * currentAy; // Low-pass filter

    // Map the potentiometer value to the desired ax range
    int potValue = (i == 0) ? receivedData.potLeft : (i == 1) ? receivedData.potMidd : receivedData.potRight;
    float desiredAy = map(potValue, 0, 4095, -16000, 16000); // Adjust to match ax range

    // Calculate PID error terms
    float error = desiredAy - smoothedAy[i];
    integralError[i] += error; // Accumulate integral error
    integralError[i] = constrain(integralError[i], -maxIntegralError, maxIntegralError); // Windup protection
    float derivative = error - previousError[i];
    previousError[i] = error;

    // PID calculation
    float pidOutput = (kp * error) + (ki * integralError[i]) + (kd * derivative);

    // Map PID output to servo position (0 to 180 degrees)
    int servoPos = constrain(map(pidOutput, -16000, 16000, 0, 180), 0, 180);

    // Update the appropriate servo
    if (i == 0) {
      servoLeft.write(servoPos);
    } else if (i == 1) {
      servoMidd.write(servoPos);
    } else if (i == 2) {
      servoRight.write(servoPos);
    }

    // Debugging output (optional)
    Serial.print("Servo ");
    Serial.print(i);
    Serial.print(" | Smoothed Ax: ");
    Serial.print(smoothedAy[i]);
    Serial.print(" | Desired Ax: ");
    Serial.print(desiredAy);
    Serial.print(" | PID Output: ");
    Serial.print(pidOutput);
    Serial.print(" | Servo Position: ");
    Serial.println(servoPos);
  }
}

// Helper function to set which MPU is active
void selectMPU(int mpuNumber) {
  // Set all AD0 pins HIGH
  digitalWrite(mpuLeftPin, HIGH);
  digitalWrite(mpuMiddPin, HIGH);
  digitalWrite(mpuRightPin, HIGH);

  // Set the selected MPU's AD0 pin LOW
  if (mpuNumber == 1) digitalWrite(mpuLeftPin, LOW);
  else if (mpuNumber == 2) digitalWrite(mpuMiddPin, LOW);
  else if (mpuNumber == 3) digitalWrite(mpuRightPin, LOW);

  delay(1);  // Allow time for the MPU6050 to stabilize
}

// Function to read data from the active MPU and calculate the servo angle
void getMPUData(int index) {
  
  mpu.getAcceleration(&mpuData[index].ax, &mpuData[index].ay, &mpuData[index].az);

  // Calculate pitch and roll using the corrected axes
  float roll = atan2(mpuData[index].ay, sqrt(mpuData[index].ax * mpuData[index].ax + mpuData[index].az * mpuData[index].az));
  float pitch = atan2(mpuData[index].ax, mpuData[index].az);

  // Store the roll and servo angle in the data structure
  RPData[index].roll = roll * 180.0 / PI;
  RPData[index].pitch = pitch * 180.0 / PI; // Convert pitch to degrees
}

// Loop through all MPUs to collect data
void getAllMPUData() {
  
  for (int i = 0; i < 3; i++) {
    selectMPU(i + 1);  // Select MPU (1-indexed)
    getMPUData(i);     // Get data and store it in the i-th struct
  }
}

// Function to handle master requests for roll pitch
void requestEvent() {
  // Send roll pitch data to the master
  WireSlave.write((uint8_t*)RPData, sizeof(RPData));
  Serial.println("Master requested data");
}

// Example of printing all MPU data
void printMPUData() {
  
  for (int i = 0; i < 3; i++) {
    Serial.print("MPU");
    Serial.print(i + 1);
    Serial.print(", Roll = ");
    Serial.print(RPData[i].roll);
    Serial.print(", Pitch = ");
    Serial.println(RPData[i].pitch);
  }
}


void loop() {
  WireSlave.update(); // Keep the I2C slave active

  // Set the currentTime
  unsigned long currentTime = millis();

  if (currentTime - previousMPUTime >= mpuInterval) {
    previousMPUTime = currentTime; // Update the timestamp
    getAllMPUData();              // Gather data from all MPUs
    printMPUData();               // Optionally print data for debugging
  }

  if (currentTime - previousStabilizeTime >= stabilizationInterval) {
    previousStabilizeTime = currentTime; // Update the timestamp
    if (currentMode == SET_POT_TO_SERVO) {
      stabilizeServos(); // Stabilize the servos based on MPU data
    }
  }

  // Perform other tasks or simply allow interrupts to work
  delay(1); // Short delay for other processes and interrupts
}
