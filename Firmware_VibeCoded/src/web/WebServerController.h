/**
 * @file WebServerController.h
 * @brief Web Server and WebSocket Controller for R6 Recon Drone
 * @author Peter
 * 
 * Manages HTTP server, WebSocket communication, and camera streaming
 */

#ifndef WEBSERVER_CONTROLLER_H
#define WEBSERVER_CONTROLLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "config.h"

// Forward declarations
struct SensorData;
struct SystemState;

class WebServerController {
private:
    AsyncWebServer* server;
    AsyncWebSocket* ws;
    
    // Camera streaming
    bool cameraStreamActive;
    uint32_t lastFrameTime;
    size_t frameCount;
    
    // Client management
    struct WebSocketClient {
        uint32_t id;
        uint32_t lastPing;
        bool authenticated;
        IPAddress clientIP;
    };
    
    std::vector<WebSocketClient> connectedClients;
    
    // HTTP handlers
    void setupRoutes();
    void handleRoot(AsyncWebServerRequest* request);
    void handleStream(AsyncWebServerRequest* request);
    void handleCapture(AsyncWebServerRequest* request);
    void handleCameraControl(AsyncWebServerRequest* request);
    void handleCameraSettings(AsyncWebServerRequest* request);
    void handleCameraSave(AsyncWebServerRequest* request);
    void handleStatus(AsyncWebServerRequest* request);
    void handleNotFound(AsyncWebServerRequest* request);
    
    // WebSocket handlers
    void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                         AwsEventType type, void* arg, uint8_t* data, size_t len);
    void handleWebSocketMessage(AsyncWebSocketClient* client, uint8_t* data, size_t len);
    void processCommand(AsyncWebSocketClient* client, JsonDocument& json);
    
    // Camera streaming
    void handleCameraStream(AsyncWebServerRequest* request);
    static void streamCameraTask(void* parameter);
    
    // Utility functions
    String getClientInfo(AsyncWebSocketClient* client);
    bool authenticateClient(AsyncWebSocketClient* client);
    void cleanupClients();
    
public:
    WebServerController();
    ~WebServerController();
    
    // Initialization
    bool begin();
    void end();
    
    // Main operations
    void handleClients();
    void broadcastTelemetry(const SensorData& sensorData, const SystemState& systemState);
    void broadcastMessage(const String& message);
    void sendToClient(uint32_t clientId, const String& message);
    
    // Camera operations
    bool startCameraStream();
    bool stopCameraStream();
    bool isCameraStreaming() const { return cameraStreamActive; }
    
    // Status and monitoring
    size_t getConnectedClients() const { return connectedClients.size(); }
    uint32_t getFrameCount() const { return frameCount; }
    float getStreamFPS() const;
    
    // Configuration
    void setStreamQuality(uint8_t quality);
    void setStreamResolution(int size);
    
    // Security
    void enableAuthentication(bool enable);
    void setAccessPassword(const String& password);
    
    // Diagnostics
    void printStatus();
    String getStatusJSON();
};

// Global command handler function
extern void handleDroneCommand(const String& command, const String& value);

#endif // WEBSERVER_CONTROLLER_H