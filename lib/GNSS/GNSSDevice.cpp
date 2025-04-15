#include "GNSSDevice.h"
#include <Arduino.h>
#include "logging.h"
GNSSDevice* GNSSDevice::instance = nullptr;

GNSSDevice::GNSSDevice(HardwareSerial* gnssSerial) : serial(gnssSerial) {
    instance = this;
}

void GNSSDevice::initialize() {
    // Try 38400 baud first, then 9600, and switch to 38400 if connected at 9600
    do {
        serial->begin(38400, SERIAL_8N1, PIN_LOCAL_GPS_RX, PIN_LOCAL_GPS_TX);
        if (gnss.begin(*serial)) break;

        delay(100);
        serial->begin(9600, SERIAL_8N1, PIN_LOCAL_GPS_RX, PIN_LOCAL_GPS_TX);
        if (gnss.begin(*serial)) {
            gnss.setSerialRate(38400);  // Switch to 38400
            delay(100);
            serial->begin(38400, SERIAL_8N1, PIN_LOCAL_GPS_RX, PIN_LOCAL_GPS_TX);  // Reinitialize serial at new rate
            gnss.begin(*serial);   // Reconnect at 38400
            break;
        }
        delay(2000);  // Wait before retrying
    } while (1);

    // Configure UART1 to output UBX only (disable NMEA)
    gnss.setUART1Output(COM_TYPE_UBX);
    
    // Set navigation frequency to 1Hz
    gnss.setNavigationFrequency(1);

    // Save configuration to flash and BBR
    gnss.saveConfiguration();

    data.isValid = false;
}

int GNSSDevice::start() {
    return DURATION_IMMEDIATELY;
}

int GNSSDevice::event() {
    return DURATION_IGNORE;
}

int GNSSDevice::timeout() {
    if (gnss.getPVT()) {  // Request UBX_NAV_PVT packet
        updateGNSSData();
    }
    return 100;  // Check again in 100ms
}

void GNSSDevice::updateGNSSData() {
    data.lastUpdate = millis();
    
    if (gnss.getGnssFixOk()) {  // Check if fix is valid
        data.latitude = gnss.getLatitude() / 1e7;   // Degrees
        data.longitude = gnss.getLongitude() / 1e7; // Degrees
        data.altitude = gnss.getAltitude() / 1e3;   // Meters
        data.isValid = true;
    } else {
        data.isValid = false;
    }
    
    if (gnss.getGroundSpeed() != 0) {
        data.speed = gnss.getGroundSpeed() / 1e3 * 3.6;  // mm/s to km/h
    }
    
    if (gnss.getHeading() != 0) {
        data.course = gnss.getHeading() / 1e5;  // Degrees
    }
    
    data.satellites = gnss.getSIV();
    
    data.hdop = gnss.getPDOP() / 100.0;  // PDOP as proxy for HDOP
}