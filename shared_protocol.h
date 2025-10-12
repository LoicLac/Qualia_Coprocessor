// ==============================================================================
// SHARED PROTOCOL - DO NOT MODIFY WITHOUT SYNCING BOTH PROJECTS
// ==============================================================================
// This file must be identical on:
// - Daisy Seed (C++ / Arduino)
// - Receiver (Raspberry Pi, ESP32, etc.)
// ==============================================================================

#pragma once
#include <cstdint>

// Binary packet structure for SPI communication
// Total size: 4 bytes (2 bytes X + 2 bytes Y)
struct __attribute__((packed)) PlotterPacket {
    uint16_t x;  // Screen X coordinate (0-719 for 720x720 display)
    uint16_t y;  // Screen Y coordinate (0-719 for 720x720 display)
};

// Protocol version (increment when changing packet structure)
#define PLOTTER_PROTOCOL_VERSION 1

// ==============================================================================
// DEBUG SYSTEM
// ==============================================================================
// Debug levels: 0=OFF, 1=ERROR, 2=INFO, 3=DEBUG
#define DEBUG_LEVEL 0

#if DEBUG_LEVEL > 0
    #define DEBUG_INIT() Serial.begin(9600); delay(2000);
#else
    #define DEBUG_INIT() ((void)0)
#endif

#if DEBUG_LEVEL >= 1
    #define DEBUG_ERROR(x) Serial.print(x)
    #define DEBUG_ERRORF(x, ...) Serial.printf(x, __VA_ARGS__)
#else
    #define DEBUG_ERROR(x) ((void)0)
    #define DEBUG_ERRORF(x, ...) ((void)0)
#endif

#if DEBUG_LEVEL >= 2
    #define DEBUG_INFO(x) Serial.println(x)
    #define DEBUG_INFOF(x, ...) Serial.printf(x, __VA_ARGS__)
#else
    #define DEBUG_INFO(x) ((void)0)
    #define DEBUG_INFOF(x, ...) ((void)0)
#endif

#if DEBUG_LEVEL >= 3
    #define DEBUG_DEBUG(x) Serial.println(x)
    #define DEBUG_DEBUGF(x, ...) Serial.printf(x, __VA_ARGS__)
#else
    #define DEBUG_DEBUG(x) ((void)0)
    #define DEBUG_DEBUGF(x, ...) ((void)0)
#endif

// Serial input helpers (only active when DEBUG_LEVEL > 0)
#if DEBUG_LEVEL > 0
    #define SERIAL_AVAILABLE() Serial.available()
    #define SERIAL_READ() Serial.read()
#else
    #define SERIAL_AVAILABLE() (0)
    #define SERIAL_READ() (0)
#endif

