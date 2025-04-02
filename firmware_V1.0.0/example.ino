/*
 * ESP32 example for managing both PZEM-004T (AC) and PZEM-017 (DC) power monitoring modules
 * using the PZEMManager library.
 * 
 * This example shows how to:
 * 1. Initialize both types of PZEM devices
 * 2. Read measurements from both devices
 * 3. Configure device parameters
 * 4. Output measurements as JSON
 * 
 * Created April 1, 2025
 */

#include <Arduino.h>
#include "PZEMManager.h"

// Define hardware serial ports for the PZEM devices
// ESP32 has 3 hardware serial ports, you can use them as needed
HardwareSerial Serial1(1);  // UART1
HardwareSerial Serial2(2);  // UART2

// PZEM-004T pin connections
#define PZEM_004T_RX_PIN 16
#define PZEM_004T_TX_PIN 17

// PZEM-017 pin connections
#define PZEM_017_RX_PIN 18
#define PZEM_017_TX_PIN 19

// Device IDs (user-defined)
#define AC_METER_ID 1
#define DC_METER_ID 2

// Create a PZEM manager instance
PZEMManager pzemManager;

// Timer variables for data reading
unsigned long lastReadTime = 0;
const unsigned long readInterval = 5000;  // Read every 5 seconds

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(500);
  
  Serial.println("PZEM Manager Example");
  Serial.println("===================");
  
  // Initialize PZEM-004T on Serial1
  Serial1.begin(9600, SERIAL_8N1, PZEM_004T_RX_PIN, PZEM_004T_TX_PIN);
  if (pzemManager.addPZEM004T(AC_METER_ID, Serial1, PZEM_004T_RX_PIN, PZEM_004T_TX_PIN, 0x01, "Main AC Supply")) {
    Serial.println("PZEM-004T (AC meter) initialized successfully");
  } else {
    Serial.println("Failed to initialize PZEM-004T");
  }
  
  // Initialize PZEM-017 on Serial2
  Serial2.begin(9600, SERIAL_8N1, PZEM_017_RX_PIN, PZEM_017_TX_PIN);
  if (pzemManager.addPZEM017(DC_METER_ID, Serial2, 0x01, "Solar DC System")) {
    Serial.println("PZEM-017 (DC meter) initialized successfully");
  } else {
    Serial.println("Failed to initialize PZEM-017");
  }
  
  // Configure alarms (optional)
  pzemManager.setPowerAlarm(AC_METER_ID, 2000);           // Set 2000W power alarm for AC meter
  pzemManager.setHighVoltageAlarm(DC_METER_ID, 3000);     // Set 30.00V high voltage alarm for DC meter
  pzemManager.setLowVoltageAlarm(DC_METER_ID, 1000);      // Set 10.00V low voltage alarm for DC meter

  // Configure shunt type for PZEM-017 (if needed)
  // 0: 100A shunt, 1: 50A shunt, 2: 200A shunt, 3: 300A shunt
  pzemManager.setShuntType(DC_METER_ID, 0);  // Use 100A shunt
}

void loop() {
  // Read and display data every 5 seconds
  if (millis() - lastReadTime >= readInterval) {
    lastReadTime = millis();
    
    // Get individual readings
    readIndividualDevices();
    
    // Alternatively, get all readings at once as JSON
    String json = pzemManager.getAllMeasurementsJSON();
    Serial.println("\nAll Device Readings (JSON):");
    Serial.println(json);
    
    Serial.println("\n-----------------------------------------\n");
  }
}

void readIndividualDevices() {
  // Read AC meter (PZEM-004T)
  PZEMMeasurement acMeasurement = pzemManager.getMeasurement(AC_METER_ID);
  
  Serial.println("AC Meter (PZEM-004T) Readings:");
  if (acMeasurement.isValid) {
    Serial.print("Voltage: ");
    Serial.print(acMeasurement.voltage, 1);
    Serial.println(" V");
    
    Serial.print("Current: ");
    Serial.print(acMeasurement.current, 3);
    Serial.println(" A");
    
    Serial.print("Power: ");
    Serial.print(acMeasurement.power, 1);
    Serial.println(" W");
    
    Serial.print("Energy: ");
    Serial.print(acMeasurement.energy, 3);
    Serial.println(" kWh");
    
    Serial.print("Frequency: ");
    Serial.print(acMeasurement.frequency, 1);
    Serial.println(" Hz");
    
    Serial.print("Power Factor: ");
    Serial.println(acMeasurement.pf, 2);
    
    Serial.print("Power Alarm: ");
    Serial.println(acMeasurement.powerAlarm ? "ON" : "OFF");
  } else {
    Serial.println("Failed to read AC meter data");
  }
  
  Serial.println();
  
  // Read DC meter (PZEM-017)
  PZEMMeasurement dcMeasurement = pzemManager.getMeasurement(DC_METER_ID);
  
  Serial.println("DC Meter (PZEM-017) Readings:");
  if (dcMeasurement.isValid) {
    Serial.print("Voltage: ");
    Serial.print(dcMeasurement.voltage, 1);
    Serial.println(" V");
    
    Serial.print("Current: ");
    Serial.print(dcMeasurement.current, 3);
    Serial.println(" A");
    
    Serial.print("Power: ");
    Serial.print(dcMeasurement.power, 1);
    Serial.println(" W");
    
    Serial.print("Energy: ");
    Serial.print(dcMeasurement.energy, 3);
    Serial.println(" kWh");
    
    Serial.print("High Voltage Alarm: ");
    Serial.println(dcMeasurement.highVoltAlarm ? "ON" : "OFF");
    
    Serial.print("Low Voltage Alarm: ");
    Serial.println(dcMeasurement.lowVoltAlarm ? "ON" : "OFF");
  } else {
    Serial.println("Failed to read DC meter data");
  }
}
