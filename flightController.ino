#include <Arduino.h>
#include <Wire.h>
#include "plane.h"
// Sensor I2C Addresses
#define ADDR_AIRSPEED 0x28
#define ADDR_ICM      0x69  // Your ICM20948 address
#define ADDR_ICM_MAG  0x0C  // ICM Magnometer
#define ADDR_BMP      0x77  // CJMCU-388 / BMP388
#define ADDR_GPS_MAG  0x0D  // HGLRC Compass (QMC5883L)

// ICM-20948 Specific Registers
#define REG_BANK_SEL    0x7F
#define REG_PWR_MGMT_1  0x06
#define REG_INT_PIN_CFG 0x0F
#define REG_USER_CTRL   0x03

// Pin for Feather V2 I2C Power
const int I2C_POWER_PIN = 2;

/**
 * Switch the ICM-20948 register bank
 */
void selectICMBank(uint8_t bank) {
    Wire.beginTransmission(ADDR_ICM);
    Wire.write(REG_BANK_SEL);
    Wire.write(bank << 4);
    Wire.endTransmission();
}

/**
 * Unlocks the 'hidden' internal magnetometer
 */
void enableICMBypass() {
    selectICMBank(0);
    
    // Wake up chip
    Wire.beginTransmission(ADDR_ICM);
    Wire.write(REG_PWR_MGMT_1);
    Wire.write(0x01); 
    Wire.endTransmission();
    delay(10);

    // Disable Master Mode & Enable Bypass
    Wire.beginTransmission(ADDR_ICM);
    Wire.write(REG_USER_CTRL);
    Wire.write(0x00); 
    Wire.endTransmission();

    Wire.beginTransmission(ADDR_ICM);
    Wire.write(REG_INT_PIN_CFG);
    Wire.write(0x02); 
    Wire.endTransmission();
    delay(10);
}

/**
 * Scans the bus for a specific address
 */
bool checkI2C(uint8_t address, const char* name, bool critical = true) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
        Serial.printf("[  OK  ] %s (0x%02X)\n", name, address);
        return true;
    } else {
        Serial.printf("[ FAIL ] %s (0x%02X)%s\n", name, address, critical ? " *CRITICAL*" : "");
        return false;
    }
}

bool isGPSAlive() {
  static unsigned long lastCheck = 0;
  static uint32_t lastCharCount = 0;
  
  // Serial1.charsProcessed() is available if using TinyGPS++
  // Otherwise, we just check if bytes are flowing:
  if (Serial1.available() > 0) {
      return true; 
  }
  
  // If no data has arrived in the last 2 seconds, the GPS is "down"
  if (millis() - lastCheck > 2000) {
    lastCheck = millis();
    return false;
  }
  return true;
}

/**
 * Runs a full diagnostic of the hardware stack.
 * Returns true only if ALL critical sensors are present.
 */
bool runPreFlightCheck() {
    bool allSystemGo = true;
    Serial.println("\n--- Starting Pre-flight Hardware Check ---");

    enableICMBypass();

    // 1. Check I2C Sensors
    if (!checkI2C(ADDR_ICM,      "IMU (ICM20948)"))   allSystemGo = false;
    if (!checkI2C(ADDR_ICM_MAG,  "IMU Mag"))  allSystemGo = false;
    if (!checkI2C(ADDR_BMP,      "Altimeter (BMP388)")) allSystemGo = false;
    if (!checkI2C(ADDR_AIRSPEED, "Airspeed (ASPD)"))  allSystemGo = false;
    if (!checkI2C(ADDR_GPS_MAG,  "GPS Compass"))      allSystemGo = false;

    // 2. Check GPS UART (Simple check if the hardware serial is initialized)
    // Note: This doesn't guarantee a satellite lock, just that the module is talking.
    if (isGPSAlive()) {
        Serial.println("[ SUCCESS ] GPS UART initialized (Serial1)");
    } else {
        Serial.println("[ FAILURE ] GPS UART NOT responding");
        allSystemGo = false;
    }

    Serial.println("------------------------------------------");
    return allSystemGo;
}

extern volatile unsigned long throttleInpPWM;
extern volatile unsigned long rudderInpPWM;
extern volatile unsigned long elevatorInpPWM;
extern volatile bool autoMode;

void setup() {
    Serial.begin(115200);
    setupPlane();

    // Feather V2: Must power the I2C bus before scanning
    pinMode(I2C_POWER_PIN, OUTPUT);
    digitalWrite(I2C_POWER_PIN, HIGH);
    delay(200); // Give sensors time to boot

    Wire.begin(22, 20); // SDA, SCL pins for your setup

    if (runPreFlightCheck()) {
        Serial.println("SYSTEM READY: Proceeding to flight logic.");
    } else {
        Serial.println("SYSTEM ERROR: Check wiring and power. Halted.");
        while(1) { 
            // Blink an onboard LED or stay here for safety
            delay(1000); 
        }
    }
}

unsigned long lastTime = 0;
void loop() {
  if((millis() - lastTime) > 500) {
    Serial.print(pwmToThrottle(throttleInpPWM));
    Serial.print(" ");
    Serial.print(pwmToRudder(rudderInpPWM));
    Serial.print(" ");
    Serial.print(pwmToElevator(elevatorInpPWM));
    Serial.print(" ");
    Serial.println(autoMode);
    lastTime = millis();
    debugInputPWM();
  }
}