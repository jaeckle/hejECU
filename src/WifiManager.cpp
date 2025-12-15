// src/WiFiManager.cpp

#include "WiFiManager.h"
#include "AppLogger.h"
#include "SystemConfig.h"
#include "WebServer.h" // Für die ESP8266WebServer-Instanz

// =========================================================
// GLOBALE VARIABLEN
// =========================================================

// Die Konfiguration wird aus SystemConfig.h importiert
extern SystemConfig_t systemConfig;

// Signalisiert dem handleWiFiMode(), dass der AP gestartet werden soll.
// Wird in SystemController.cpp verwendet.
bool startApMode = false;

// Konstanten
const char *AP_SSID = "ECU_CONFIG";
const char *AP_PASSWORD = "password";
const long WIFI_TIMEOUT_MS = 10000; // 10 Sekunden Timeout

// =========================================================
// HILFSFUNKTIONEN
// =========================================================

/**
 * @brief Definiert und startet den Access Point.
 */
void startAPMode()
{
    LOG(LOG_INFO, "WiFi: Starte AP-Modus (%s)", AP_SSID);

    // AP-Einstellungen festlegen
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(
        IPAddress(192, 168, 4, 1),// Lokale IP
        IPAddress(192, 168, 4, 1),// Gateway
        IPAddress(255, 255, 255, 0)// Subnetzmaske
    );

    // AP starten
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    LOG(LOG_WARN, "WiFi: AP gestartet. IP: %s", WiFi.softAPIP().toString().c_str());
    LOG(LOG_WARN, "WiFi: Mit AP verbinden, um Konfiguration zu ändern.");
    
    // Webserver-Status setzen (definiert in WebServer.cpp)
    setServerStarted(true); 
}

// =========================================================
// SERVICE-IMPLEMENTIERUNGEN
// =========================================================

/**
 * @brief Versucht, eine Verbindung zum konfigurierten STA-Netzwerk herzustellen.
 * @return true, wenn die Verbindung erfolgreich war; false sonst.
 */
bool setupWiFi()
{
    // WLAN-Modus
    WiFi.mode(WIFI_STA);
    
    LOG(LOG_INFO, "WiFi: Versuche Verbindung zu %s...", systemConfig.wifi_ssid);
    
    // Verbindung versuchen
    WiFi.begin(systemConfig.wifi_ssid, systemConfig.wifi_password);

    unsigned long start_time = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start_time < WIFI_TIMEOUT_MS)
    {
        delay(500);
        // Wir verwenden hier delay() und nicht yield(), da wir uns noch im Setup befinden
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        return true;
    }
    
    // Verbindung fehlgeschlagen
    return false;
}

/**
 * @brief Verwaltet den WLAN-Zustand (STA/AP). Wird im Loop aufgerufen.
 */
void handleWiFiMode()
{
    // Logik für den AP-Start: Nur wenn AP signalisiert wird UND noch kein Server läuft.
    if (startApMode && WiFi.getMode() != WIFI_AP)
    {
        startAPMode();
        startApMode = false; // Flag zurücksetzen
    }
}