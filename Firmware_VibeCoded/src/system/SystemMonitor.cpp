/**
 * @file SystemMonitor.cpp
 * @brief System Monitor Implementation
 * @author Peter
 */

#include "SystemMonitor.h"
#include "esp_system.h"
#include "esp_heap_caps.h"

SystemMonitor::SystemMonitor()
    : initialized(false), lastUpdateTime(0), bootTime(0),
      freeHeap(0), freePsram(0), cpuUsage(0.0f), temperature(0.0f),
      taskCount(0), stackHighWaterMark(0) {
}

bool SystemMonitor::begin() {
    DEBUG_PRINTLN("Initializing System Monitor...");
    
    bootTime = millis();
    lastUpdateTime = bootTime;
    initialized = true;
    
    DEBUG_PRINTLN("System Monitor initialized");
    return true;
}

void SystemMonitor::update() {
    if (!initialized) return;
    
    uint32_t currentTime = millis();
    if (currentTime - lastUpdateTime < SYSTEM_STATUS_UPDATE_MS) return;
    
    // Update memory statistics
    freeHeap = ESP.getFreeHeap();
    freePsram = ESP.getFreePsram();
    
    // Update CPU usage (simplified)
    cpuUsage = 0.0f; // Would need more complex implementation
    
    // Update temperature (if available)
    #if TEMPERATURE_MONITOR_ENABLED
    temperature = temperatureRead();
    #endif
    
    // Update task count
    taskCount = uxTaskGetNumberOfTasks();
    
    // Update stack high water mark
    stackHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    
    lastUpdateTime = currentTime;
}

void SystemMonitor::checkSystemHealth() {
    if (!initialized) return;
    
    // Check memory levels
    if (freeHeap < 10000) {
        DEBUG_PRINTLN("WARNING: Low heap memory!");
    }
    
    if (freePsram < 1000000) {
        DEBUG_PRINTLN("WARNING: Low PSRAM memory!");
    }
    
    // Check stack usage
    if (stackHighWaterMark < 500) {
        DEBUG_PRINTLN("WARNING: Low stack space!");
    }
    
    // Check temperature
    if (temperature > 70.0f) {
        DEBUG_PRINTLN("WARNING: High temperature!");
    }
}

uint32_t SystemMonitor::getUptime() const {
    return millis() - bootTime;
}

void SystemMonitor::printSystemInfo() {
    DEBUG_PRINTLN("=== System Information ===");
    DEBUG_PRINTF("Uptime: %lu ms\n", getUptime());
    DEBUG_PRINTF("Free Heap: %lu bytes\n", freeHeap);
    DEBUG_PRINTF("Free PSRAM: %lu bytes\n", freePsram);
    DEBUG_PRINTF("Tasks: %lu\n", taskCount);
    DEBUG_PRINTF("Stack HWM: %lu\n", stackHighWaterMark);
    DEBUG_PRINTF("CPU Usage: %.1f%%\n", cpuUsage);
    
    #if TEMPERATURE_MONITOR_ENABLED
    DEBUG_PRINTF("Temperature: %.1f°C\n", temperature);
    #endif
    
    // Chip information
    DEBUG_PRINTF("Chip Model: %s\n", ESP.getChipModel());
    DEBUG_PRINTF("Chip Revision: %d\n", ESP.getChipRevision());
    DEBUG_PRINTF("CPU Frequency: %lu MHz\n", ESP.getCpuFreqMHz());
    DEBUG_PRINTF("Flash Size: %lu bytes\n", ESP.getFlashChipSize());
    DEBUG_PRINTF("Flash Speed: %lu Hz\n", ESP.getFlashChipSpeed());
    
    DEBUG_PRINTLN("==========================");
}

String SystemMonitor::getSystemInfoJSON() {
    String json = "{";
    json += "\"uptime\":" + String(getUptime()) + ",";
    json += "\"freeHeap\":" + String(freeHeap) + ",";
    json += "\"freePsram\":" + String(freePsram) + ",";
    json += "\"cpuUsage\":" + String(cpuUsage) + ",";
    json += "\"temperature\":" + String(temperature) + ",";
    json += "\"tasks\":" + String(taskCount) + ",";
    json += "\"stackHWM\":" + String(stackHighWaterMark) + ",";
    json += "\"chipModel\":\"" + String(ESP.getChipModel()) + "\",";
    json += "\"cpuFreq\":" + String(ESP.getCpuFreqMHz()) + ",";
    json += "\"flashSize\":" + String(ESP.getFlashChipSize());
    json += "}";
    return json;
}