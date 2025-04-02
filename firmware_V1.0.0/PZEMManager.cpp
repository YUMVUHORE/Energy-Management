/*
 * PZEMManager.cpp
 * 
 * Implementation of the PZEMManager class for managing PZEM-004T and PZEM-017
 * power monitoring modules with a single ESP32 controller.
 * 
 * Created April 1, 2025
 */

#include "PZEMManager.h"
#include <ArduinoJson.h>

PZEMManager::PZEMManager() {
  _deviceCount = 0;
  
  // Initialize device array
  for (uint8_t i = 0; i < MAX_PZEM_DEVICES; i++) {
    _devices[i].isInitialized = false;
    _devices[i].device = nullptr;
    
    // Initialize measurements
    _measurements[i].isValid = false;
  }
}

PZEMManager::~PZEMManager() {
  // Clean up device objects
  for (uint8_t i = 0; i < _deviceCount; i++) {
    if (_devices[i].isInitialized) {
      if (_devices[i].type == PZEM_004T) {
        delete static_cast<PZEM004Tv30*>(_devices[i].device);
      } else if (_devices[i].type == PZEM_017) {
        delete static_cast<PZEM017v1*>(_devices[i].device);
      }
    }
  }
}

bool PZEMManager::addPZEM004T(uint8_t id, HardwareSerial& port, uint8_t rxPin, uint8_t txPin, uint8_t addr, const String& name) {
  // Check if we already have too many devices
  if (_deviceCount >= MAX_PZEM_DEVICES) {
    return false;
  }
  
  // Check if ID is already in use
  if (getDeviceIndex(id) != -1) {
    return false;
  }
  
  // Create new PZEM004T device
  PZEM004Tv30* device = new PZEM004Tv30(port, rxPin, txPin, addr);
  if (!device) {
    return false;
  }
  
  // Add to the device list
  _devices[_deviceCount].id = id;
  _devices[_deviceCount].type = PZEM_004T;
  _devices[_deviceCount].address = addr;
  _devices[_deviceCount].device = static_cast<void*>(device);
  _devices[_deviceCount].port = &port;
  _devices[_deviceCount].isInitialized = true;
  _devices[_deviceCount].name = name;
  
  _deviceCount++;
  
  return true;
}

bool PZEMManager::addPZEM017(uint8_t id, HardwareSerial& port, uint8_t addr, const String& name) {
  // Check if we already have too many devices
  if (_deviceCount >= MAX_PZEM_DEVICES) {
    return false;
  }
  
  // Check if ID is already in use
  if (getDeviceIndex(id) != -1) {
    return false;
  }
  
  // Create new PZEM017 device
  PZEM017v1* device = new PZEM017v1(&port, addr);
  if (!device) {
    return false;
  }
  
  // Add to the device list
  _devices[_deviceCount].id = id;
  _devices[_deviceCount].type = PZEM_017;
  _devices[_deviceCount].address = addr;
  _devices[_deviceCount].device = static_cast<void*>(device);
  _devices[_deviceCount].port = &port;
  _devices[_deviceCount].isInitialized = true;
  _devices[_deviceCount].name = name;
  
  _deviceCount++;
  
  return true;
}

int PZEMManager::getDeviceIndex(uint8_t id) {
  for (uint8_t i = 0; i < _deviceCount; i++) {
    if (_devices[i].id == id) {
      return i;
    }
  }
  return -1;
}

PZEMMeasurement PZEMManager::getMeasurement(uint8_t id) {
  int index = getDeviceIndex(id);
  if (index == -1) {
    PZEMMeasurement invalid;
    invalid.isValid = false;
    return invalid;
  }
  
  // Update and return measurement
  updateMeasurement(index);
  return _measurements[index];
}

PZEMMeasurement* PZEMManager::getAllMeasurements() {
  updateAllMeasurements();
  return _measurements;
}

bool PZEMManager::updateMeasurement(uint8_t index) {
  if (index >= _deviceCount || !_devices[index].isInitialized) {
    return false;
  }
  
  PZEMMeasurement& measurement = _measurements[index];
  
  // Initialize measurement as invalid until we get values
  measurement.isValid = false;
  
  if (_devices[index].type == PZEM_004T) {
    PZEM004Tv30* device = static_cast<PZEM004Tv30*>(_devices[index].device);
    
    // Get the values from the PZEM004T
    measurement.voltage = device->voltage();
    measurement.current = device->current();
    measurement.power = device->power();
    measurement.energy = device->energy();
    measurement.frequency = device->frequency();
    measurement.pf = device->pf();
    measurement.powerAlarm = device->getPowerAlarm();
    
    // These are not applicable to PZEM004T
    measurement.highVoltAlarm = false;
    measurement.lowVoltAlarm = false;
    
    // Check if values are valid (not NaN)
    if (!isnan(measurement.voltage) && !isnan(measurement.current) && 
        !isnan(measurement.power) && !isnan(measurement.energy)) {
      measurement.isValid = true;
    }
  } 
  else if (_devices[index].type == PZEM_017) {
    PZEM017v1* device = static_cast<PZEM017v1*>(_devices[index].device);
    
    // Get the values from the PZEM017
    measurement.voltage = device->voltage();
    measurement.current = device->current();
    measurement.power = device->power();
    measurement.energy = device->energy();
    measurement.highVoltAlarm = device->isHighvoltAlarmOn();
    measurement.lowVoltAlarm = device->isLowvoltAlarmOn();
    
    // These are not applicable to PZEM017
    measurement.frequency = 0;
    measurement.pf = 0;
    measurement.powerAlarm = false;
    
    // Check if values are valid (not NaN)
    if (!isnan(measurement.voltage) && !isnan(measurement.current) && 
        !isnan(measurement.power) && !isnan(measurement.energy)) {
      measurement.isValid = true;
    }
  }
  
  return measurement.isValid;
}

void PZEMManager::updateAllMeasurements() {
  for (uint8_t i = 0; i < _deviceCount; i++) {
    updateMeasurement(i);
  }
}

String PZEMManager::getMeasurementJSON(uint8_t id) {
  int index = getDeviceIndex(id);
  if (index == -1) {
    return "{\"error\":\"Device not found\"}";
  }
  
  // Update measurement
  updateMeasurement(index);
  
  // Create JSON document
  StaticJsonDocument<512> doc;
  
  doc["id"] = _devices[index].id;
  doc["name"] = _devices[index].name;
  doc["type"] = _devices[index].type == PZEM_004T ? "PZEM-004T" : "PZEM-017";
  doc["address"] = _devices[index].address;
  
  if (_measurements[index].isValid) {
    doc["voltage"] = _measurements[index].voltage;
    doc["current"] = _measurements[index].current;
    doc["power"] = _measurements[index].power;
    doc["energy"] = _measurements[index].energy;
    
    if (_devices[index].type == PZEM_004T) {
      doc["frequency"] = _measurements[index].frequency;
      doc["pf"] = _measurements[index].pf;
      doc["powerAlarm"] = _measurements[index].powerAlarm;
    } else if (_devices[index].type == PZEM_017) {
      doc["highVoltageAlarm"] = _measurements[index].highVoltAlarm;
      doc["lowVoltageAlarm"] = _measurements[index].lowVoltAlarm;
    }
  } else {
    doc["error"] = "Measurement failed";
  }
  
  String output;
  serializeJson(doc, output);
  return output;
}

String PZEMManager::getAllMeasurementsJSON() {
  // Update all measurements
  updateAllMeasurements();
  
  // Create JSON document
  StaticJsonDocument<4096> doc;
  JsonArray devices = doc.createNestedArray("devices");
  
  for (uint8_t i = 0; i < _deviceCount; i++) {
    JsonObject device = devices.createNestedObject();
    
    device["id"] = _devices[i].id;
    device["name"] = _devices[i].name;
    device["type"] = _devices[i].type == PZEM_004T ? "PZEM-004T" : "PZEM-017";
    device["address"] = _devices[i].address;
    
    if (_measurements[i].isValid) {
      device["voltage"] = _measurements[i].voltage;
      device["current"] = _measurements[i].current;
      device["power"] = _measurements[i].power;
      device["energy"] = _measurements[i].energy;
      
      if (_devices[i].type == PZEM_004T) {
        device["frequency"] = _measurements[i].frequency;
        device["pf"] = _measurements[i].pf;
        device["powerAlarm"] = _measurements[i].powerAlarm;
      } else if (_devices[i].type == PZEM_017) {
        device["highVoltageAlarm"] = _measurements[i].highVoltAlarm;
        device["lowVoltageAlarm"] = _measurements[i].lowVoltAlarm;
      }
    } else {
      device["error"] = "Measurement failed";
    }
  }
  
  String output;
  serializeJson(doc, output);
  return output;
}

bool PZEMManager::setAddress(uint8_t id, uint8_t newAddr) {
  int index = getDeviceIndex(id);
  if (index == -1) {
    return false;
  }
  
  bool success = false;
  
  if (_devices[index].type == PZEM_004T) {
    PZEM004Tv30* device = static_cast<PZEM004Tv30*>(_devices[index].device);
    success = device->setAddress(newAddr);
  } else if (_devices[index].type == PZEM_017) {
    PZEM017v1* device = static_cast<PZEM017v1*>(_devices[index].device);
    success = device->setAddress(newAddr);
  }
  
  if (success) {
    _devices[index].address = newAddr;
  }
  
  return success;
}

bool PZEMManager::resetEnergy(uint8_t id) {
  int index = getDeviceIndex(id);
  if (index == -1) {
    return false;
  }
  
  if (_devices[index].type == PZEM_004T) {
    PZEM004Tv30* device = static_cast<PZEM004Tv30*>(_devices[index].device);
    return device->resetEnergy();
  } else if (_devices[index].type == PZEM_017) {
    PZEM017v1* device = static_cast<PZEM017v1*>(_devices[index].device);
    return device->resetEnergy();
  }
  
  return false;
}

bool PZEMManager::setPowerAlarm(uint8_t id, uint16_t watts) {
  int index = getDeviceIndex(id);
  if (index == -1 || _devices[index].type != PZEM_004T) {
    return false;
  }
  
  PZEM004Tv30* device = static_cast<PZEM004Tv30*>(_devices[index].device);
  return device->setPowerAlarm(watts);
}

bool PZEMManager::setHighVoltageAlarm(uint8_t id, uint16_t volts) {
  int index = getDeviceIndex(id);
  if (index == -1 || _devices[index].type != PZEM_017) {
    return false;
  }
  
  PZEM017v1* device = static_cast<PZEM017v1*>(_devices[index].device);
  return device->setHighvoltAlarm(volts);
}

bool PZEMManager::setLowVoltageAlarm(uint8_t id, uint16_t volts) {
  int index = getDeviceIndex(id);
  if (index == -1 || _devices[index].type != PZEM_017) {
    return false;
  }
  
  PZEM017v1* device = static_cast<PZEM017v1*>(_devices[index].device);
  return device->setLowvoltAlarm(volts);
}

bool PZEMManager::setShuntType(uint8_t id, uint16_t type) {
  int index = getDeviceIndex(id);
  if (index == -1 || _devices[index].type != PZEM_017) {
    return false;
  }
  
  PZEM017v1* device = static_cast<PZEM017v1*>(_devices[index].device);
  return device->setShuntType(type);
}

uint8_t PZEMManager::getDeviceCount() {
  return _deviceCount;
}
