#pragma once
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Replace TinyGPSPlus
#include "device.h"

struct GNSSData {
    bool isValid = false;
    unsigned long lastUpdate = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude = 0.0;
    float speed = 0.0;
    float course = 0.0;
    uint32_t satellites = 0;
    float hdop = 0.0;
};

class GNSSDevice {
private:
    static GNSSDevice* instance;
    SFE_UBLOX_GNSS gnss;
    HardwareSerial* serial;
    GNSSData data;

    void initialize();
    int start();
    int event();
    int timeout();
    void updateGNSSData();

    static void initializeWrapper() {
        if (instance) instance->initialize();
    }
    static int startWrapper() {
        return instance ? instance->start() : DURATION_NEVER;
    }
    static int eventWrapper() {
        return instance ? instance->event() : DURATION_IGNORE;
    }
    static int timeoutWrapper() {
        return instance ? instance->timeout() : DURATION_NEVER;
    }

public:
    GNSSDevice(HardwareSerial* gnssSerial);
    device_t device = {
        initializeWrapper,
        startWrapper,
        eventWrapper,
        timeoutWrapper
    };

    bool isDataValid() { return data.isValid; }
    double getLatitude() { return data.latitude; }
    double getLongitude() { return data.longitude; }
    float getAltitude() { return data.altitude; }
    float getSpeed() { return data.speed; }
    float getCourse() { return data.course; }
    uint32_t getSatellites() { return data.satellites; }
    float getHdop() { return data.hdop; }
};