/*
 * PZEMManager.h
 * 
 * A comprehensive library for managing PZEM-004T and PZEM-017 power monitoring modules
 * with a single ESP32 controller.
 * 
 * This library handles multiple PZEM devices connected to different serial ports
 * and provides a unified interface for retrieving measurements.
 * 
 * Created April 1, 2025
 * 
 * Based on the original PZEM004Tv30 and PZEM017v1 libraries by Jakub Mandula
 * and Maxz Maxzerker respectively.
 * 
 */

#ifndef PZEM_MANAGER_H
#define PZEM_MANAGER_H

#include <Arduino.h>
#include "PZEM004Tv30.h"
#include "PZEM017v1.h"

// Maximum number of PZEM devices that can be managed (adjust as needed)
#define MAX_PZEM_DEVICES 10

// Device types
enum PZEMType {
  PZEM_004T,  // AC Power Meter
  PZEM_017    // DC Power Meter
};

// Structure to hold measurement data from any PZEM device
struct PZEMMeasurement {
  float voltage;        // V
  float current;        // A
  float power;          // W
  float energy;         // kWh
  float frequency;      // Hz (AC only)
  float pf;             // Power Factor (AC only)
  bool highVoltAlarm;   // High voltage alarm (DC only)
  bool lowVoltAlarm;    // Low voltage alarm (DC only)
  bool powerAlarm;      // Power alarm (AC only)
  bool isValid;         // Indicates if the measurement was successful
};

// Individual device representation
struct PZEMDevice {
  uint8_t id;           // User-assigned ID for the device
  PZEMType type;        // Type of PZEM device
  uint8_t address;      // Modbus address of the device
  void* device;         // Pointer to the actual device object
  HardwareSerial* port; // Serial port used by this device
  bool isInitialized;   // Flag to indicate if the device is initialized
  String name;          // Optional name for the device
};

class PZEMManager {
public:
  PZEMManager();
  ~PZEMManager();

  // Add a PZEM-004T device on a given hardware serial port
  bool addPZEM004T(uint8_t id, HardwareSerial& port, uint8_t rxPin, uint8_t txPin, uint8_t addr = PZEM_DEFAULT_ADDR, const String& name = "");
  
  // Add a PZEM-017 device on a given hardware serial port
  bool addPZEM017(uint8_t id, HardwareSerial& port, uint8_t addr = PZEM_DEFAULT_ADDR, const String& name = "");

  // Get measurements from a specific device by ID
  PZEMMeasurement getMeasurement(uint8_t id);
  
  // Get all measurements (returns an array of measurements)
  PZEMMeasurement* getAllMeasurements();
  
  // Get measurement as JSON string for a specific device
  String getMeasurementJSON(uint8_t id);
  
  // Get all measurements as JSON string
  String getAllMeasurementsJSON();
  
  // Set address for a device
  bool setAddress(uint8_t id, uint8_t newAddr);
  
  // Reset energy counter for a device
  bool resetEnergy(uint8_t id);
  
  // Set power alarm for PZEM-004T
  bool setPowerAlarm(uint8_t id, uint16_t watts);
  
  // Set high voltage alarm for PZEM-017
  bool setHighVoltageAlarm(uint8_t id, uint16_t volts);
  
  // Set low voltage alarm for PZEM-017
  bool setLowVoltageAlarm(uint8_t id, uint16_t volts);

  // Set shunt type for PZEM-017
  bool setShuntType(uint8_t id, uint16_t type);
  
  // Get device index by ID
  int getDeviceIndex(uint8_t id);
  
  // Get number of devices
  uint8_t getDeviceCount();

private:
  PZEMDevice _devices[MAX_PZEM_DEVICES];
  uint8_t _deviceCount;
  PZEMMeasurement _measurements[MAX_PZEM_DEVICES];
  
  // Update measurements for all devices
  void updateAllMeasurements();
  
  // Update measurement for a specific device
  bool updateMeasurement(uint8_t index);
};

#endif // PZEM_MANAGER_H
