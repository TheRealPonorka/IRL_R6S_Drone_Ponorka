/**
 * @file SystemMonitor.h
 * @brief System Monitor for R6 Recon Drone
 * @author Peter
 */

#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <Arduino.h>
#include "config.h"

class SystemMonitor {
private:
    bool initialized;
    uint32_t lastUpdateTime;
    uint32_t bootTime;
    
    // System stats
    uint32_t freeHeap;
    uint32_t freePsram;
    float cpuUsage;
    float temperature;
    
    // Task monitoring
    uint32_t taskCount;
    uint32_t stackHighWaterMark;
    
public:
    SystemMonitor();
    
    // Initialization
    bool begin();
    
    // Monitoring
    void update();
    void checkSystemHealth();
    
    // Statistics
    uint32_t getUptime() const;
    uint32_t getFreeHeap() const { return freeHeap; }
    uint32_t getFreePsram() const { return freePsram; }
    float getCpuUsage() const { return cpuUsage; }
    float getTemperature() const { return temperature; }
    
    // System info
    void printSystemInfo();
    String getSystemInfoJSON();
    
    // Status
    bool isInitialized() const { return initialized; }
};

#endif // SYSTEM_MONITOR_H