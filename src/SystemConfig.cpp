// src/SystemConfig.cpp

#include "SystemConfig.h"
#include "AppLogger.h"
#include <LittleFS.h>
#include <ArduinoJson.h>

// NEUE INCLUDES FÜR FEHLERBEHEBUNG
#include "IgnitionManager.h" // <<< Fuer loadIgnitionConfig und updateSpeedCalibrationFactor
#include "WiFiManager.h" // Für loadWiFiConfig

// =========================================================
// 1. GLOBALE VARIABLEN DEFINITIONEN
// =========================================================

// Definition der globalen Variablen (initialer Wert: INFO)
volatile LogLevel activeLogLevel = LOG_INFO;
const char *config_path = "/config.json";

// =========================================================
// 2. MODUL CONFIG LOADER (Minimal Implementierung)
// =========================================================

bool loadModuleConfigs(JsonVariant json) 
{
    // Ruft die spezifischen Ladefunktionen auf
    bool success_wifi = loadWiFiConfig(json);
    bool success_ignition = loadIgnitionConfig(json); // <<< Jetzt sichtbar

    return success_wifi && success_ignition;
}

void saveModuleConfigs(JsonObject json) 
{
    // ... Implementierung später
}

// =========================================================
// 3. CONFIG FILE HANDLING
// =========================================================

bool loadConfig()
{
    // ... (Logik zum Lesen der Datei) ...

    if (!loadDataFromJson(config_path))
    {
        // Fehler beim Laden, versuche Standardeinstellungen zu speichern
        LOG(LOG_ERROR, "Config: Konfiguration konnte nicht geladen werden.");
        // Speichere default config (wenn implementiert)
        return false;
    }

    // NACH dem Laden der Configs, Aktionen ausführen, die die Config benötigen
    // updateSpeedCalibrationFactor() (aus IgnitionManager)
    updateSpeedCalibrationFactor(); // <<< Jetzt sichtbar

    LOG(LOG_INFO, "Config: Konfiguration erfolgreich geladen.");
    return true;
}


bool loadDataFromJson(const char *path)
{
    // ... (Platzhalter-Logik) ...
    
    // Annahme: Laden erfolgreich, jetzt werden die Module aufgerufen:
    DynamicJsonDocument doc(1024);
    // DeserializeJson...
    
    // Die Aufrufe der Modul-Konfigurationsfunktionen
    loadModuleConfigs(doc.as<JsonVariant>());

    // updateSpeedCalibrationFactor(); // Nur in loadConfig() aufrufen

    return true;
}


bool saveConfig()
{
    // ... (Platzhalter) ...
    return true;
}