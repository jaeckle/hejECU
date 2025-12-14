// WebServer.cpp

#include "WebServer.h" 
#include <LittleFS.h>
#include <ArduinoOTA.h> 
#include <ESP.h>
#include <ESP8266mDNS.h>    
#include <ArduinoJson.h>
#include <algorithm> 
#include "AppLogger.h" // AppLogger einbinden

// LOG MACRO HIER DEFINIERT (jetzt über AppLogger.h)
#define LOG(level, format, ...) app_log(level, __FILE__, __LINE__, __LINE__, format, ##__VA_ARGS__) 

// Externe Deklarationen (aus CoreDefs.h und AppLogger.h)
extern ESP8266WebServer server;
extern volatile LogLevel activeLogLevel;
extern String wifiSsid; 
extern String wifiPassword; 
extern String apSsid; 
extern String apPassword; 
extern bool startApMode;
extern bool serverStarted;
extern volatile unsigned long max_loop_duration_us;
extern volatile bool is_critical_latency_active;
extern volatile unsigned long last_critical_max_duration_us;
extern volatile unsigned long last_critical_timestamp_ms;

// Externe Variablen aus AppLogger.h
extern const char *LOG_FILENAME;

// Externe Structs
extern struct RpmConfig rpmConfig;
extern struct IgnitionTimingConfig timingConfig;
extern struct IgnitionCoilConfig coilConfig;
extern IgnitionMap2D ignitionMap2D; 

// Externe Funktionen
extern bool saveConfig();
extern bool loadDataFromJson(const char* jsonString);
extern const char* FALLBACK_CURVE_JSON;
extern int getIgnitionRpmSafe();
extern int calculateBilinearAngle(int rpm, int tps); 
extern int pinNameToCode(String pinName);
extern void setPinState(int pinCode, bool targetState);
extern float getSensorTpsRawSafe();
extern float getSensorBattVSafe();
extern float getSensorTempKopfSafe();
extern float getSensorTempKopfSafe();
extern float getSensorSpeedSafe();


// =========================================================
// SICHERHEIT & SYSTEM
// =========================================================

// isAuthorized() ist in main.cpp definiert.

void handleRestart() {
    if (!isAuthorized()) return server.requestAuthentication();
    LOG(LOG_WARN, "Systemneustart durch Web-Interface angefordert."); 
    server.send(200, "text/plain", F("Neustart des Systems..."));
    delay(1000); 
    ESP.restart();
}

// logCriticalEvent Implementierung wurde nach AppLogger.cpp verschoben (wird über extern verfügbar)


// =========================================================
// OTA SETUP
// =========================================================

void setup_ota() {
    // ... Implementierung unverändert ...
    if (WiFi.status() != WL_CONNECTED) { LOG(LOG_WARN, "OTA nicht gestartet: Kein WLAN."); return; }
    MDNS.begin(apSsid.c_str());
    // ...
    ArduinoOTA.begin();
    LOG(LOG_INFO, "OTA Initialisiert.");
}


// =========================================================
// HANDLER (Ansicht/Aktion)
// =========================================================

// handleRoot mit Link-Aktivierung
void handleRoot() {
    if (!isAuthorized()) return server.requestAuthentication();
    LOG(LOG_INFO, "WEB: Root-Anfrage empfangen."); // <--- NEUER INFO LOG

    int rpm = getIgnitionRpmSafe();
    
    // Verwendung der sicheren Getter
    int tps_raw = (int)getSensorTpsRawSafe();
    float batt_v = getSensorBattVSafe();
    float temp_kopf = getSensorTempKopfSafe();
    int speed = (int)getSensorSpeedSafe();
    
    // START Chunked/Streaming Response
    server.sendHeader(F("Cache-Control"), F("no-cache, no-store, must-revalidate"));
    server.sendHeader(F("Pragma"), F("no-cache"));
    server.sendHeader(F("Expires"), F("-1"));
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, F("text/html"), F("")); // Start der Antwort senden

    String content = F("<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'></head><body>");
    server.sendContent(content);

    server.sendContent(F("<h1>🏍️ ECU Dashboard v1.2 (Version 0.0.100)</h1>"));
    server.sendContent(F("<h2>System Status</h2>"));
    
    if (startApMode) {
        server.sendContent(F("<p style='color: orange;'>⚠️ Momentan im **ACCESS POINT** Modus.</p>"));
        server.sendContent(F("<p>AP IP: 192.168.4.1</p>"));
    } else {
        server.sendContent(F("<p style='color: green;'>✅ Verbunden über **STA** Modus.</p>"));
        String ip_content = F("<p>IP: ") + WiFi.localIP().toString() + F("</p>");
        server.sendContent(ip_content);
    }

    String status_content;
    status_content += F("<p>Aktuelle RPM: "); status_content += String(rpm); status_content += F("</p>");
    status_content += F("<p>Max. Loop-Dauer (letzte Sekunde): "); status_content += String(max_loop_duration_us); status_content += F(" µs</p>");
    status_content += F("<p>Freier Heap: "); status_content += String(ESP.getFreeHeap()); server.sendContent(F(" Bytes</p>"));
    server.sendContent(status_content);
    
    // Anzeige der Sensorwerte
    String sensor_content = F("<h2>Echtzeit-Sensordaten</h2><p>");
    sensor_content += F("TPS (Raw): ") + String(tps_raw) + F(" | ");
    sensor_content += F("Geschw.: ") + String(speed) + F(" km/h | ");
    sensor_content += F("Batterie: ") + String(batt_v, 1) + F(" V | ");
    sensor_content += F("Temperatur: ") + String(temp_kopf, 1) + F(" °C</p>");
    server.sendContent(sensor_content);
    
    server.sendContent(F("<h2>Navigation</h2><ul>"));
    server.sendContent(F("<li><a href='/viewlog'>📜 Log-Ansicht & Log-Level konfigurieren</a></li>"));
    server.sendContent(F("<li><a href='/config'>🌐 WLAN Konfiguration</a></li>"));
    server.sendContent(F("<li><a href='/map_editor'>📈 Zündkennfeld Editor (2D)</a></li>")); // <--- LINK REAKTIVIERT
    server.sendContent(F("<li><a href='/rpmconfig'>⚙️ RPM Setup & Spule</a></li>"));
    server.sendContent(F("<li><a href='/restart'>🔄 System neu starten</a></li></ul>"));
    
    server.sendContent(F("</body></html>"));
    server.sendContent(""); // Endet den Chunked Transfer
}


void handleNotFound() { /* ... Implementierung unverändert ... */ }
void handleClearLog() { /* ... Implementierung unverändert ... */ }
void handleViewLog() { /* ... Implementierung unverändert ... */ }
void handleSetLogLevel() { /* ... Implementierung unverändert ... */ }
void handleConfig() { /* ... Implementierung unverändert ... */ }
void handleSaveConfig() { /* ... Implementierung unverändert ... */ }
void handleRpmConfig() { /* ... Implementierung unverändert ... */ }
void handleSaveRpmConfig() { /* ... Implementierung unverändert ... */ }
void handleRestoreCurve() { /* ... Implementierung unverändert ... */ }


// --- NEUE 2D MAP HANDLER IMPLEMENTIERUNG ---

void handleMapApiData() { /* ... Implementierung unverändert ... */ }
void handleMapApiUpdate() { /* ... Implementierung unverändert ... */ }
String generateMapEditorHtml() { /* ... Implementierung unverändert ... */ return F("<!DOCTYPE html><html><head>... </p></body></html>"); }
void handleMapEditor() { /* ... Implementierung unverändert ... */ }


// =========================================================
// ROUTEN KONFIGURATION (Stack-Optimierung durch Aufteilung)
// =========================================================

void registerSystemRoutes() { /* ... Implementierung unverändert ... */ }
void registerRpmConfigRoutes() { /* ... Implementierung unverändert ... */ }
void registerLogRoutes() { /* ... Implementierung unverändert ... */ }

void registerMapRoutes() {
    // 2D Map Routen (Ausgelagert, um Stack-Smashing in setup_server_routes zu verhindern)
    server.on("/map_editor", HTTP_GET, handleMapEditor); 
    server.on("/api/map_data", HTTP_GET, handleMapApiData);
    server.on("/api/map_update", HTTP_POST, handleMapApiUpdate);
    LOG(LOG_INFO, "Map Routen registriert.");
}

void setup_server_routes() {
    // Registrierung in separaten Funktionen
    registerSystemRoutes();
    registerRpmConfigRoutes();
    registerLogRoutes();
    registerMapRoutes();

    // Fallback
    server.onNotFound(handleNotFound);

    // WICHTIG: KEIN server.begin() HIER! Der Server wird verzögert in loop() gestartet.
    serverStarted = true;
    LOG(LOG_INFO, "HTTP Routen konfiguriert.");
}