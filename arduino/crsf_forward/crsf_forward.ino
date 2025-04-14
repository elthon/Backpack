#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <Adafruit_NeoPixel.h>

// UID for ESP-NOW receiver (converted from 51,154,51,154,232,99)
uint8_t receiverUID[] = {0x32, 0x9A, 0x33, 0x9A, 0xE8, 0x63}; // First byte adjusted to even

// UART configuration for CRSF
#define RX_PIN 16
#define TX_PIN 17

// RGB LED configuration (assuming WS2812 on PIN47)
#define RGB_PIN 48
#define NUM_PIXELS 1
Adafruit_NeoPixel rgb(NUM_PIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

// Buffer for CRSF data
uint8_t crsfBuffer[64];
uint8_t bufferIndex = 0;

// ESP-NOW peer info
esp_now_peer_info_t peerInfo;

// CRC8 lookup table for CRSF
const uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

// RGB status functions
void setRGBColor(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}

void blinkRGB(uint8_t r, uint8_t g, uint8_t b, uint32_t onTime, uint32_t offTime = 0) {
  setRGBColor(r, g, b);
  delay(onTime);
  setRGBColor(0, 0, 0);
  if (offTime > 0) delay(offTime);
}

// Callback for ESP-NOW send status
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    blinkRGB(0, 255, 0, 100); // Green flash for success
  } else {
    blinkRGB(255, 0, 0, 100); // Red flash for failure
  }
}

// Calculate CRSF CRC8
uint8_t crsf_frame_crc(const uint8_t *buf) {
  uint8_t crc = 0;
  for (uint8_t i = 2; i < buf[1] + 1; i++) {
    crc = crc8tab[crc ^ buf[i]];
  }
  return crc;
}

void setup() {
  // Initialize RGB LED
  rgb.begin();
  rgb.setBrightness(50); // Adjust brightness (0-255)
  setRGBColor(0, 255, 0); // Green during initialization
  
  // Initialize CRSF UART
  Serial.begin(460800); //, SERIAL_8N1, RX_PIN, TX_PIN);
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    blinkRGB(255, 0, 0, 500, 500); // Red blink for ESP-NOW init failure
    while (1); // Halt
  }

  blinkRGB(0, 255, 0, 500, 500); // Red blink for ESP-NOW init failure

  // Register send callback
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer using UID as MAC address
  memcpy(peerInfo.peer_addr, receiverUID, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    blinkRGB(255, 0, 0, 500, 500); // Red blink for peer add failure
    while (1); // Halt
  }

  // Initialization complete
  delay(2000); // Show green for 2 seconds
  setRGBColor(0, 0, 0);
}

void loop() {
  static unsigned long lastBlink = 0;
  // Slow blue blink while waiting for CRSF data
  if (millis() - lastBlink >= 1000) {
    blinkRGB(0, 0, 255, 500, 500); // Blue blink every 1s
    lastBlink = millis();
  }

  // Read CRSF data from UART
  while (Serial.available()) {
    uint8_t byte = Serial.read();
    
    // Check for CRSF sync byte
    if (byte == 0xC8 && bufferIndex == 0) {
      crsfBuffer[bufferIndex++] = byte;
    } 
    else if (bufferIndex > 0) {
      crsfBuffer[bufferIndex++] = byte;
      
      // Check if we have length byte
      if (bufferIndex == 2) {
        if (crsfBuffer[1] > 62) { // Invalid length
          bufferIndex = 0;
        }
      }
      
      // Complete frame received
      if (bufferIndex > 2 && bufferIndex == crsfBuffer[1] + 2) {
        // Verify CRC
        if (crsf_frame_crc(crsfBuffer) == crsfBuffer[bufferIndex - 1]) {
          blinkRGB(128, 0, 128, 200, 200); // Purple flash for valid CRSF
          
          // Send via ESP-NOW using UID
          esp_err_t result = esp_now_send(receiverUID, crsfBuffer, bufferIndex);
          
          if (result != ESP_OK) {
            blinkRGB(255, 0, 0, 100); // Red flash for send error
          }else{
            blinkRGB(165, 255, 0, 100); // Red flash for send error
          }
        } else {
          blinkRGB(255, 165, 0, 100); // Orange flash for CRC failure
        }
        
        bufferIndex = 0; // Reset buffer
      }
      
      // Prevent buffer overflow
      if (bufferIndex >= 64) {
        bufferIndex = 0;
      }
    }
  }
}