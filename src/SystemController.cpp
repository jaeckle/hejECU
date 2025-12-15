// src/SystemController.cpp

#include "SystemController.h"
#include "AppLogger.h"
#include "SystemConfig.h"
#include "IgnitionManager.h"
#include "SensorManager.h"
#include "WiFiManager.h"
#include "WebServer.h"
#include "SystemStats.h"
#include <LittleFS.h>
#include <Ticker.h>// Für Ticker-Definitionen
#include <ArduinoOTA.h> // Für ArduinoOTA::handle()

// Definition der globalen Konfigurationsinstanz (Aktion 145.1)
SystemConfig_t systemConfig = {
    "YourSSID",// default wifi_ssid
    "YourPass",// default wifi_password
    // ... andere Standardwerte
};

// =========================================================
// 1. GLOBALE DEFS UND VARIABLEN (vom Controller verwaltet)
// =========================================================

// Externe Deklarationen (aus anderen Modulen)
extern ESP8266WebServer server;// Definiert in WebServer.cpp
extern Ticker advanceTicker;// Definiert in SystemStats.cpp
extern Ticker debugTicker;// Definiert in SystemStats.cpp
extern bool startApMode;// Definiert in WiFiManager.cpp

// Globale Variablen des Controllers
unsigned long lastSystemMaintenance = 0;
const unsigned long MAINTENANCE_INTERVAL_MS = 1000;


// =========================================================
// 2. SETUP PHASE
// =========================================================

void setupModules()
{
    // 1. Initialisiere Dateisystem und Konfiguration
    if (!LittleFS.begin())
    {
        LOG(LOG_WARN, "LittleFS formatiert.");
        LittleFS.format();
    }
    if (!loadConfig()) // Ruft SystemConfig::loadConfig() auf
    {
        LOG(LOG_ERROR, "Config: Fehler beim Laden der Konfiguration.");
    }
    
    // 2. Initialisiere Hardware und Sensoren
    setupIgnition();// GPIOs, Interrupts (IgnitionManager)
    initializeSensors();// SensorManager

    // 3. WLAN Setup und Verbindungsversuch (Warteschleife ist nach setupWiFi() verschoben)
    if (setupWiFi()) // setupWiFi() gibt true zurück bei erfolgreichem STA-Mode
    {
        // STA Mode (Verbunden)
        LOG(LOG_INFO, "WiFi: Erfolgreich verbunden. IP: %s", WiFi.localIP().toString().c_str());
        
        // Server und Ticker starten
        setup_server_routes();// WebServer
        server.begin();// WebServer
        setup_ota();// WebServer
        
        // Ticker starten: Zündungsberechnung läuft asynchron 
        advanceTicker.attach_ms(50, updateIgnitionParameters); // 20 Hz
        debugTicker.attach(1, printLoopStatus);// 1 Hz (SystemStats)

    }
    else
    {
        // AP Mode (Fehlgeschlagen)
        LOG(LOG_WARN, "WiFi: Verbindung fehlgeschlagen. Starte AP-Modus.");
        startApMode = true;// Signalisiert WiFiManager, AP zu starten
        setup_server_routes(); // Routen sind auch im AP-Modus nötig
    }
    
    LOG(LOG_INFO, "Setup: Module initialisiert. Heap: %d Bytes", ESP.getFreeHeap());
}


// =========================================================
// 3. LOOP PHASE
// =========================================================

// Implementierung von loadWiFiConfig (Aktion 147.1)
bool loadWiFiConfig(JsonVariant json) {
    // Hier würde später die Logik zum Laden der WLAN-Konfiguration stehen
    // aus der JSON-Datei in die systemConfig-Struktur.
    return true; 
}

void handleNetworkLoop()
{
    // Die gesamte Netzwerk-Verwaltungslogik wird hier gesteuert.

    // Wenn im AP-Modus gestartet, muss der AP-Modus im Loop verwaltet werden.
    // WiFiManager.cpp enthält die Logik, um zwischen STA und AP zu wechseln.
    handleWiFiMode(); 
    
    // OTA-Handling
    ArduinoOTA.handle();

    // Webserver-Client-Handling
    server.handleClient();
}


void handleSystemMaintenance()
{
    // Führt periodische Systemprüfungen und Wartungsaufgaben aus
    unsigned long current_time = millis();

    if (current_time - lastSystemMaintenance >= MAINTENANCE_INTERVAL_MS)
    {
        checkHeapHealth();// SystemStats
        
        // Optional: Hier könnten weitere periodische Aufgaben folgen (z.B. Log-Rotation)
        
        lastSystemMaintenance = current_time;
    }
}