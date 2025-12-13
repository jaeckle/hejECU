/*
 * Copyright (c) 2025 "hej und Gemini" (Contributor: [Ihr GitHub-Username])
 *
 * This software is licensed under the GNU General Public License v3.0 (GPL-3.0).
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation.
 *
 * A copy of the license is provided in the LICENSE file in the project root.
 */

 // WebServer.cpp
#include "WebServer.h" 
#include <LittleFS.h>
#include <ArduinoOTA.h> 
#include <ESP.h>
#include <ESP8266mDNS.h>    
#include <ArduinoJson.h>
#include <algorithm> 

// Externe Deklarationen (aus CoreDefs.h)
extern ESP8266WebServer server;
extern volatile LogLevel activeLogLevel;
extern String apPassword;
extern bool startApMode;
extern const char *LOG_FILENAME;
extern std::vector<String> startupLogBuffer;
extern bool serverStarted;
extern volatile unsigned long max_loop_duration_us; 
extern volatile bool is_critical_latency_active;
extern volatile unsigned long last_critical_max_duration_us;
extern volatile unsigned long last_critical_timestamp_ms;

// Externe Structs
extern struct RpmConfig rpmConfig;
extern struct IgnitionTimingConfig timingConfig;
extern struct IgnitionCoilConfig coilConfig;
extern std::vector<MapPoint> ignitionMap;

// Externe Funktionen
extern bool saveConfig();
extern bool loadDataFromJson(const char* jsonString);
extern const char* FALLBACK_CURVE_JSON;
extern int getIgnitionRpmSafe();
extern int getAdvanceAngle2D(int rpm, int tps);
extern int pinNameToCode(String pinName);
extern void setPinState(int pinCode, bool targetState);
extern float getSensorTpsRawSafe();
extern float getSensorBattVSafe();
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

void logCriticalEvent(unsigned long maxDuration, unsigned long timestamp, const char* status) {
    if (!LittleFS.begin()) {
        LOG(LOG_ERROR, "LittleFS ist nicht bereit, kritisches Log kann nicht geschrieben werden.");
        return;
    }
    
    File logFile = LittleFS.open(LOG_FILENAME, "a");
    if (!logFile) {
        LOG(LOG_ERROR, "Konnte kritisches Log-Datei nicht öffnen/erstellen.");
        return;
    }
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "[%lu ms] [CRIT] %s - Max Loop: %lu us\n", 
             timestamp, status, maxDuration);
             
    logFile.print(buffer);
    logFile.close();
    LOG(LOG_WARN, "Kritisches Event protokolliert: %s", status);
}


// =========================================================
// OTA SETUP
// =========================================================

void setup_ota() {
    if (WiFi.status() != WL_CONNECTED) { LOG(LOG_WARN, "OTA nicht gestartet: Kein WLAN."); return; }
    MDNS.begin(apSsid.c_str());
    ArduinoOTA.setHostname(apSsid.c_str());
    ArduinoOTA.setPassword(apPassword.c_str());

    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "Sketch";
        } else { // U_SPIFFS
            type = "Filesystem";
            LittleFS.end(); 
        }
        LOG(LOG_INFO, "Starte OTA-Update: %s", type.c_str());
    });

    ArduinoOTA.onEnd([]() {
        LOG(LOG_INFO, "\nOTA-Update abgeschlossen. Neustart...");
    });

    ArduinoOTA.onError([](ota_error_t error) {
        LOG(LOG_ERROR, "OTA-Fehler [%d]: %s", error, "Fehlertyp");
    });
    
    ArduinoOTA.begin();
    LOG(LOG_INFO, "OTA Initialisiert.");
}


// =========================================================
// HANDLER (Ansicht/Aktion)
// =========================================================

void handleRoot() {
    if (!isAuthorized()) return server.requestAuthentication();

    int rpm = getIgnitionRpmSafe();
    
    // Verwendung der sicheren Getter
    int tps_raw = (int)getSensorTpsRawSafe();
    float batt_v = getSensorBattVSafe();
    float temp_kopf = getSensorTempKopfSafe();
    int speed = (int)getSensorSpeedSafe();
    
    String content = F("<html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width, initial-scale=1'></head><body>");
    
    content += F("<h1>🏍️ ECU Dashboard v1.2 (Version 0.0.001)</h1>");
    content += F("<h2>System Status</h2>");
    
    if (startApMode) {
        content += F("<p style='color: orange;'>⚠️ Momentan im **ACCESS POINT** Modus.</p>");
        content += F("<p>WLAN Verbindung fehlgeschlagen. Bitte WLAN-Konfiguration prüfen.</p>");
    } else {
        content += F("<p style='color: green;'>✅ Verbunden über **STA** Modus.</p>");
        content += F("<p>IP: "); content += WiFi.localIP().toString(); content += F("</p>");
    }

    content += F("<p>Aktuelle RPM: "); content += String(rpm); content += F("</p>");
    content += F("<p>Max. Loop-Dauer (letzte Sekunde): "); content += String(max_loop_duration_us); content += F(" µs</p>");
    content += F("<p>Freier Heap: "); content += String(ESP.getFreeHeap()); content += F(" Bytes</p>");
    
    // Anzeige der Sensorwerte
    content += F("<h2>Echtzeit-Sensordaten</h2><p>");
    content += F("TPS (Raw): "); content += String(tps_raw); content += F(" | ");
    content += F("Geschw.: "); content += String(speed); content += F(" km/h | ");
    content += F("Batterie: "); content += String(batt_v, 1); content += F(" V | ");
    content += F("Temperatur: "); content += String(temp_kopf, 1); content += F(" °C</p>");
    
    content += F("<h2>Navigation</h2>");
    content += F("<ul>");
    content += F("<li><a href='/viewlog'>📜 Log-Ansicht & Log-Level konfigurieren</a></li>");
    content += F("<li><a href='/config'>🌐 WLAN Konfiguration</a></li>");
    content += F("<li><a href='/curve'>📈 Zündkurve</a></li>");
    content += F("<li><a href='/rpmconfig'>⚙️ RPM Setup</a></li>");
    content += F("<li><a href='/restart'>🔄 System neu starten</a></li>");
    content += F("</ul>");
    
    content += F("</body></html>");
    server.send(200, "text/html; charset=utf-8", content);
}

void handleNotFound() {
    if (!isAuthorized()) return server.requestAuthentication();
    String message = F("File Not Found\n\n");
    message += F("URI: ");
    message += server.uri();
    message += F("\nMethod: ");
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    server.send(404, "text/plain", message);
}

// Handler zum Löschen des kritischen Logs
void handleClearLog() {
    if (!isAuthorized()) return server.requestAuthentication();

    bool finalSuccess = false;

    if (LittleFS.remove(LOG_FILENAME)) { 
        LOG(LOG_INFO, "Logdatei erfolgreich entfernt."); 
        File logFile = LittleFS.open(LOG_FILENAME, "w");
        if (logFile) {
            logFile.close();
            LOG(LOG_INFO, "Logdatei erfolgreich neu erstellt.");
            finalSuccess = true;
        } else {
            LOG(LOG_ERROR, "Logdatei entfernt, aber Neuanlage von %s fehlgeschlagen.", LOG_FILENAME);
        }
    } else {
        LOG(LOG_WARN, "LittleFS.remove() fehlgeschlagen. Versuche, die Datei zu leeren und neu zu erstellen...");
        File logFile = LittleFS.open(LOG_FILENAME, "w"); 
        
        if (logFile) {
            logFile.close(); 
            LOG(LOG_INFO, "Log-Datei erfolgreich auf 0 Byte gekürzt/neu erstellt.");
            finalSuccess = true;
        } else {
            LOG(LOG_ERROR, "Kritischer Fehler: Konnte %s weder entfernen noch kürzen/neu erstellen.", LOG_FILENAME);
        }
    }

    if (finalSuccess) {
        server.send(200, F("text/plain"), F("Kritische Log-Datei gelöscht/geleert."));
    } else {
        server.send(500, F("text/plain"), F("Fehler beim Löschvorgang."));
    }
}

// NEUE Version: Integriert Log-Level Konfiguration und Log-Anzeige
void handleViewLog() {
    if (!isAuthorized()) return server.requestAuthentication();

    String content = F("<html><head><meta charset='UTF-8'></head><body>");
    content += F("<h1>📜 Log-Ansicht und Konfiguration</h1>");
    
    // Log-Level Konfigurationsformular
    content += F("<h2>🎚️ Log-Level Konfiguration (Live & Persistent)</h2>");
    content += F("<form method='post' action='/setloglevel'>"); 
    
    // Log-Level Selectbox
    content += F("<p>Aktueller Log-Level: "); content += String((int)activeLogLevel); content += F("</p>");
    content += F("Neuer Log-Level: <select name='level' id='logLevelSelect'>");
    content += F("<option value='0'"); if (activeLogLevel == LOG_DEBUG) content += F(" selected"); content += F(">0: DEBUG (Sehr ausführlich)</option>");
    content += F("<option value='1'"); if (activeLogLevel == LOG_INFO) content += F(" selected"); content += F(">1: INFO (Normaler Betrieb)</option>");
    content += F("<option value='2'"); if (activeLogLevel == LOG_WARN) content += F(" selected"); content += F(">2: WARNUNG (Auffälligkeiten)</option>");
    content += F("<option value='3'"); if (activeLogLevel == LOG_ERROR) content += F(" selected"); content += F(">3: FEHLER (Kritische Fehler)</option>");
    content += F("</select>");
    content += F("<br><br>");
    
    // Buttons
    content += F("<button type='submit' name='action' value='apply_only' style='background: #ffcc00; margin-right: 10px;'>🧪 Im RAM anwenden (Live)</button>");
    content += F("<button type='submit' name='action' value='save_all'>✅ Speichern auf Flash & Neustart</button>");
    content += F("</form><hr>");

    // Bestehende Log-Ausgabe 
    content += F("<h2>📜 Gespeicherte Logs & RAM-Puffer</h2><pre style='background: #eee; padding: 10px;'>");
    
    // RAM Puffer
    content += F("--- STARTUP LOG BUFFER (RAM) ---\n");
    for (const String& log : startupLogBuffer) {
        content += log;
        content += "\n";
    }
    
    // FLASH Log
    content += F("\n--- CRITICAL LOG (FLASH: ");
    if (LittleFS.exists(LOG_FILENAME)) {
        File logFile = LittleFS.open(LOG_FILENAME, "r");
        if (logFile) {
            content += String(logFile.size()); content += F(" Bytes) ---\n");
            // Lesepuffer, um große Dateien zu vermeiden
            char buffer[64];
            while (logFile.available()) {
                size_t bytesRead = logFile.readBytes(buffer, sizeof(buffer) - 1);
                buffer[bytesRead] = '\0';
                content += buffer;
            }
            logFile.close();
        } else {
            content += F("FEHLER beim Öffnen) ---\n");
        }
    } else {
        content += F("NICHT VORHANDEN) ---\n");
    }
    
    content += F("</pre><p><a href='/'>Zurück zur Steuerung</a> | <a href='/clearlog'>Löschen des FLASH Logs</a></p></body></html>");
    server.send(200, "text/html; charset=utf-8", content);
}

// NEUER Handler: Verarbeitet die POST-Anfrage zur Log-Level-Änderung
void handleSetLogLevel() {
    if (!isAuthorized()) return server.requestAuthentication();

    if (!server.hasArg(F("level")) || !server.hasArg(F("action"))) {
        server.send(400, "text/plain", F("Bad Request: Level oder Action fehlt."));
        return;
    }
    
    int newLevel = server.arg(F("level")).toInt();
    String action = server.arg(F("action")); 
    
    // Level im RAM sofort aktualisieren
    noInterrupts();
    activeLogLevel = (LogLevel)constrain(newLevel, 0, 3);
    interrupts();
    
    LOG(LOG_INFO, "Log-Level im RAM auf %d angewendet.", (int)activeLogLevel);

    if (action == F("save_all")) {
        // AKTION 1: SPEICHERN UND NEUSTART
        if (saveConfig()) {
            LOG(LOG_WARN, "Log-Level gespeichert. Systemneustart in Kürze...");
            server.send(200, F("text/plain"), F("Log-Level gespeichert. Neustart..."));
            delay(1000);
            ESP.restart();
            return;
        } else {
            LOG(LOG_ERROR, "Speichern der Config nach Log-Level-Änderung fehlgeschlagen.");
            server.send(500, F("text/plain"), F("Fehler beim Speichern."));
            return;
        }
    }
    
    // AKTION 2: LIVE ANWENDEN (apply_only) oder Fallback:
    server.sendHeader(F("Location"), F("/viewlog"));
    server.send(303); // HTTP 303 See Other (Standard für POST-Redirect)
}


// --- Fehlende Konfigurationshandler (Implementierung) ---

void handleConfig() {
    if (!isAuthorized()) return server.requestAuthentication();

    String html = F("<html><head><meta charset='UTF-8'></head><body><h1>🌐 WLAN Konfiguration</h1>");
    
    html += F("<h2>Client Mode (STA)</h2>");
    html += F("<form method='post' action='/saveconfig'>");
    html += F("STA SSID: <input type='text' name='ssid' maxlength='32' value='");
    html += wifiSsid; html += F("' required><br>");
    html += F("STA Passwort: <input type='password' name='password' minlength='8' value='");
    html += wifiPassword; html += F("' required><br><br>");
    
    html += F("<h2>Access Point Mode (AP - Fallback)</h2>");
    html += F("AP SSID (Hostname): <input type='text' name='ap_ssid' maxlength='32' value='");
    html += apSsid; html += F("' required><br>");
    html += F("AP Passwort: <input type='password' name='ap_password' minlength='8' value='");
    html += apPassword; html += F("' required><br><br>");
    
    html += F("<input type='submit' value='Speichern und Neustart' style='padding:10px;'>");
    html += F("</form><p><a href='/'>Zurück zum Dashboard</a></p></body></html>");
    
    server.send(200, "text/html; charset=utf-8", html);
}

void handleSaveConfig() {
    if (!isAuthorized()) return server.requestAuthentication();

    if (!server.hasArg(F("ssid")) || !server.hasArg(F("password")) || !server.hasArg(F("ap_ssid")) || !server.hasArg(F("ap_password"))) {
        server.send(400, "text/plain", F("Bad Request: Alle Felder erforderlich."));
        return;
    }
    
    String newSsid = server.arg(F("ssid"));
    String newPass = server.arg(F("password"));
    String newApSsid = server.arg(F("ap_ssid"));
    String newApPass = server.arg(F("ap_password"));
    
    if (newPass.length() < 8) {
         server.send(400, "text/plain", F("Fehler: STA Passwort muss mindestens 8 Zeichen haben."));
         return;
    }

    noInterrupts();
    wifiSsid = newSsid;
    wifiPassword = newPass;
    apSsid = newApSsid;
    apPassword = newApPass;
    interrupts();

    if (saveConfig()) { 
        LOG(LOG_INFO, "WLAN Config gespeichert. Neustart wird durchgeführt.");
        server.send(200, F("text/plain"), F("Gespeichert. Neustart...")); 
        ESP.restart(); 
    } 
    else { 
        LOG(LOG_ERROR, "Speichern der WLAN-Config fehlgeschlagen.");
        server.send(500, F("text/plain"), F("Fehler beim Speichern.")); 
    }
}

void handleCurveConfig() {
    if (!isAuthorized()) return server.requestAuthentication();

    String html = F("<html><head><meta charset='UTF-8'><title>Zündkurven-Konfiguration</title></head><body>");
    html += F("<h1>📈 Zündkurven-Konfiguration</h1>");
    html += F("<p>Hier können Sie die Zündkennfeld-Punkte bearbeiten. Änderungen werden erst nach 'Speichern & Neustart' permanent.</p>");
    html += F("<form method='post' action='/savecurve'>");

    // --- TABELLE START ---
    html += F("<table border='1' cellpadding='5'>");
    html += F("<tr><th>#</th><th>Drehzahl (RPM)</th><th>TPS (0-1023)</th><th>Winkel (° v. OT)</th><th>Aktion</th></tr>");

    std::vector<MapPoint> currentMap;
    noInterrupts();
    currentMap = ignitionMap;
    interrupts();
    
    // Sortierung der Map-Punkte
    if (currentMap.size() > 1) {
        std::sort(currentMap.begin(), currentMap.end(), [](const MapPoint& a, const MapPoint& b) {
            if (a.rpm != b.rpm) return a.rpm < b.rpm;
            return a.tps < b.tps;
        });
    }

    for (size_t i = 0; i < currentMap.size(); ++i) {
        String idx = String(i);
        html += F("<tr><td>"); html += idx; html += F("</td>");
        html += F("<td><input type='number' name='rpm_"); html += idx;
        html += F("' value='"); html += String(currentMap[i].rpm); html += F("' min='0' required></td>");
        html += F("<td><input type='number' name='tps_"); html += idx;
        html += F("' value='"); html += String(currentMap[i].tps); html += F("' min='0' max='1023' required></td>");
        html += F("<td><input type='number' name='angle_");
        html += idx; html += F("' value='"); html += String(currentMap[i].angle); html += F("' min='-20' max='30' required></td>");
        html += F("<td><button type='submit' name='delete_idx' value='"); html += idx;
        html += F("'>Löschen</button></td></tr>");
    }

    String new_idx = String(currentMap.size());
    html += F("<tr><td>"); html += new_idx; html += F(" (Neu)</td>");
    html += F("<td><input type='number' name='rpm_"); html += new_idx;
    html += F("' value='' min='0' placeholder='RPM' ></td>");
    html += F("<td><input type='number' name='tps_"); html += new_idx;
    html += F("' value='' min='0' max='1023' placeholder='TPS' ></td>");
    html += F("<td><input type='number' name='angle_"); html += new_idx;
    html += F("' value='' min='-20' max='30' placeholder='Winkel' ></td>");
    html += F("<td></td></tr>"); 
    
    html += F("</table>");
    html += F("<br><h2>Aktionen</h2>");
    
    html += F("<button type='submit' name='action' value='apply_only' style='background: #ffcc00; margin-right: 10px;'>🧪 Im RAM Anwenden (Test)</button>");
    html += F("<button type='submit' name='action' value='save_all'>✅ Speichern auf Flash & Neustart</button>");
    
    html += F("</form>"); 

    html += F("<h2>Werkseinstellungen</h2>");
    html += F("<p>Stellt die Zündkurve auf den unveränderlichen Firmware-Standard zurück und startet neu.</p>");
    html += F("<form method='post' action='/restorecurve'>");
    html += F("<button type='submit'>⚠️ Standard-Kurve Wiederherstellen</button>");
    html += F("</form>");

    html += F("<p><a href='/'>Zurück zum Dashboard</a></p></body></html>");
    
    server.send(200, "text/html; charset=utf-8", html);
}

void handleSaveCurve() {
    if (!isAuthorized()) return server.requestAuthentication();

    String action = server.arg(F("action"));
    
    String delete_idx_str = server.arg(F("delete_idx"));
    if (delete_idx_str.length() > 0) {
        int delete_index = delete_idx_str.toInt();
        
        noInterrupts();
        if (delete_index >= 0 && (size_t)delete_index < ignitionMap.size()) {
            ignitionMap.erase(ignitionMap.begin() + delete_index);
            LOG(LOG_INFO, "Kennfeldpunkt %d gelöscht.", delete_index);
        }
        interrupts();
        
        server.sendHeader(F("Location"), F("/curve"));
        server.send(303);
        return;
    }

    std::vector<MapPoint> newMap;
    
    for (int i = 0; i < server.args(); i++) {
        String argName = server.argName(i);
        
        if (argName.startsWith(F("rpm_"))) {
            int index = argName.substring(4).toInt();
            String rpmStr = server.arg(argName);
            String tpsStr = server.arg(F("tps_") + String(index));
            String angleStr = server.arg(F("angle_") + String(index));
            
            if (rpmStr.isEmpty() || tpsStr.isEmpty() || angleStr.isEmpty()) {
                if (action != F("add_new") || (size_t)index != ignitionMap.size()) {
                    continue;
                }
            }
            
            if (rpmStr.isEmpty() || tpsStr.isEmpty() || angleStr.isEmpty()) {
                if (action == F("add_new") && (size_t)index == ignitionMap.size()) {
                     server.send(400, F("text/plain"), F("Fehler: Neuer Punkt muss RPM, TPS und Winkel haben."));
                     return;
                }
            }

            int rpm = rpmStr.toInt();
            int tps = tpsStr.toInt();
            int angle = angleStr.toInt();

            if (rpm < 0 || rpm > 15000 || tps < 0 || tps > 1023 || angle < -20 || angle > 30) {
                LOG(LOG_ERROR, "Ungültige Kennfeldwerte (%d RPM, %d TPS, %d Angle).", rpm, tps, angle);
                server.send(400, F("text/plain"), F("Fehler: Ungültige Werte für RPM/TPS/Winkel."));
                return;
            }

            newMap.push_back({rpm, tps, angle});
        }
    }

    // Kurve übernehmen (in den RAM)
    noInterrupts();
    ignitionMap = newMap;
    interrupts();
    
    if (action == F("save_all")) {
        LOG(LOG_INFO, "Zündkennfeld in RAM übernommen. Speichere und starte neu.");
        if (saveConfig()) {
            server.send(200, F("text/plain"), F("Kennfeld gespeichert. Neustart..."));
            delay(1000);
            ESP.restart(); 
            return;
        } else {
            server.send(500, F("text/plain"), F("Fehler beim Speichern der Konfiguration."));
            return;
        }
    } 
    
    String responseHtml = F("<html><head><meta http-equiv='refresh' content='2;url=/curve'></head><body><h1>Erfolg!</h1><p>Konfiguration im RAM angewendet. Seite wird in 2 Sekunden neu geladen.</p><p><a href='/curve'>Manuell fortfahren</a></p></body></html>");
    server.send(200, "text/html; charset=utf-8", responseHtml);
}

void handleRestoreCurve() {
    if (!isAuthorized()) return server.requestAuthentication();

    LOG(LOG_WARN, "Wiederherstellung des Standard-Zündkennfelds angefordert.");

    if (loadDataFromJson(FALLBACK_CURVE_JSON)) {
        if (saveConfig()) {
            server.send(200, F("text/plain"), F("Standard-Kennfeld erfolgreich wiederhergestellt. Neustart..."));
            delay(1000);
            ESP.restart();
            return;
        }
    }
    
    server.send(500, F("text/plain"), F("Fehler: Konnte Standard-Kennfeld nicht speichern."));
}

void handleRpmConfig() {
    if (!isAuthorized()) return server.requestAuthentication();

    String html = F("<html><head><meta charset='UTF-8'></head><body><h1>RPM & Zündungs-Timing Konfiguration</h1>");
    html += F("<form method='post' action='/saverpmconfig'>");
    
    // --- 1. RPM Sensor / Timing (Trigger Wheel) ---
    html += F("<h2>1. RPM Sensor / Timing (Trigger Wheel)</h2>");
    html += F("<p><b>Pin:</b> D1 (fest verdrahtet)</p>");
    html += F("Impulse pro Umdrehung: <input type='number' name='pulses' min='1' max='20' value='");
    html += String(rpmConfig.pulses_per_revolution); html += F("' required><br>");
    html += F("Sensor Typ: <select name='is_digital'><option value='1' "); html += rpmConfig.is_digital ? F("selected") : F(""); html += F(">Digital</option><option value='0' "); html += !rpmConfig.is_digital ? F("selected") : F(""); html += F(">Analog</option></select><br><br>");
    
    // --- 2. Statische Zündungs-Justierung (Mechanik) ---
    html += F("<h2>2. Statische Zündungs-Justierung (Mechanik)</h2>");
    html += F("Trigger Offset (&deg; v.OT): <input type='number' name='offset' min='0' max='360' value='");
    html += String(timingConfig.trigger_offset_deg); html += F("' required><br>");
    html += F("TDC Feinjustierung (&deg;): <input type='number' name='tdc' step='0.1' value='");
    html += String(timingConfig.TDC_adjust_deg, 1); html += F("' required><br><br>");

    // --- 3. Zündspule (Elektrische Kalibrierung) ---
    html += F("<h2>3. Zündspule (Elektrische Kalibrierung)</h2>");
    html += F("Primärwiderstand Spule (Rp in &Omega;): <input type='number' name='rp' step='0.1' value='");
    html += String(coilConfig.primary_resistance_ohm, 1); html += F("' min='0.1' required><br>");
    html += F("Externer Vorwiderstand (Rext in &Omega;): <input type='number' name='rext' step='0.1' value='");
    html += String(coilConfig.external_resistance_ohm, 1); html += F("' min='0.0' required><br>");
    html += F("Primärinduktivität (Lp in mH): <input type='number' name='lp' step='0.1' value='");
    html += String(coilConfig.primary_inductance_mH, 1); html += F("' min='0.1' required><br>");
    html += F("Ziel-Primärstrom (I_target in A): <input type='number' name='itarget' step='0.1' value='");
    html += String(coilConfig.target_current_A, 1); html += F("' min='1.0' required><br>");
    html += F("Statische Dwell-Zeit (Fallback in ms): <input type='number' name='dwell' step='0.1' value='");
    html += String(coilConfig.fixed_dwell_ms, 1); html += F("' min='1.0' required><br><br>");
    
    // Aktionen
    html += F("<button type='submit' name='action' value='apply_only' style='background: #ffcc00; margin-right: 10px;'>🧪 Im RAM Anwenden (Test)</button>");
    html += F("<button type='submit' name='action' value='save_all'>✅ Speichern auf Flash & Neustart</button>");
    
    html += F("</form><p><a href='/'>Zurück</a></p></body></html>");
    server.send(200, "text/html; charset=utf-8", html);
}

void handleSaveRpmConfig() {
    if (!isAuthorized()) return server.requestAuthentication();

    String action = server.arg(F("action"));
    
    // Daten auslesen und validieren
    int pulses = server.arg(F("pulses")).toInt();
    int offset = server.arg(F("offset")).toInt();
    float tdc = server.arg(F("tdc")).toFloat();
    // bool is_digital = server.arg(F("is_digital")).toInt() == 1; // Muss im Struct-Update beachtet werden

    // Spulendaten auslesen
    float rp = server.arg(F("rp")).toFloat();
    float rext = server.arg(F("rext")).toFloat();
    float lp = server.arg(F("lp")).toFloat();
    float itarget = server.arg(F("itarget")).toFloat();
    float dwell = server.arg(F("dwell")).toFloat();


    if (pulses <= 0 || pulses > 20 || offset < 0 || offset > 360 || rp <= 0 || lp <= 0 || itarget <= 0 || dwell <= 0) {
         server.send(400, "text/plain", F("Fehler: Ungültige Werte für RPM/Timing oder Spulenparameter."));
         return;
    }

    // On-the-Fly Update der globalen VOLATILE Variablen
    noInterrupts();
    rpmConfig.pulses_per_revolution = pulses;
    // rpmConfig.is_digital = is_digital; // Muss im Formular angepasst werden
    timingConfig.trigger_offset_deg = offset;
    timingConfig.TDC_adjust_deg = tdc;

    // Zündspulen Configs aktualisieren
    coilConfig.primary_resistance_ohm = rp;
    coilConfig.external_resistance_ohm = rext;
    coilConfig.primary_inductance_mH = lp;
    coilConfig.target_current_A = itarget;
    coilConfig.fixed_dwell_ms = dwell;
    
    interrupts();
    
    LOG(LOG_INFO, "RPM/Timing/Spule Config im RAM aktualisiert. Rp+Rext: %.1f Ohm", rp + rext);

    if (action == F("save_all")) {
        if (saveConfig()) {
            server.send(200, F("text/plain"), F("Konfiguration gespeichert. Neustart..."));
            delay(1000);
            ESP.restart(); 
            return;
        } else {
            server.send(500, F("text/plain"), F("Fehler beim Speichern der Konfiguration."));
            return;
        }
    } 
    
    // apply_only oder Fallback: Auto-Refresh zur Konfigurationsseite
    String responseHtml = F("<html><head><meta http-equiv='refresh' content='2;url=/rpmconfig'></head><body><h1>Erfolg!</h1><p>Konfiguration im RAM angewendet. Seite wird in 2 Sekunden neu geladen.</p><p><a href='/rpmconfig'>Manuell fortfahren</a></p></body></html>");
    server.send(200, "text/html; charset=utf-8", responseHtml);
}


// =========================================================
// ROUTEN KONFIGURATION
// =========================================================

void setup_server_routes() {
    server.on(F("/"), handleRoot);
    server.on(F("/restart"), handleRestart);
    
    // Konfigurations-Routen
    server.on(F("/config"), handleConfig); 
    server.on(F("/saveconfig"), handleSaveConfig); 
    server.on(F("/curve"), handleCurveConfig);
    server.on(F("/savecurve"), handleSaveCurve);
    server.on(F("/restorecurve"), handleRestoreCurve);
    server.on(F("/rpmconfig"), handleRpmConfig);
    server.on(F("/saverpmconfig"), handleSaveRpmConfig);

    // Log-Routen
    server.on(F("/viewlog"), HTTP_GET, handleViewLog); 
    server.on(F("/setloglevel"), HTTP_POST, handleSetLogLevel); 
    server.on(F("/clearlog"), handleClearLog); 
    
    // Fallback
    server.onNotFound(handleNotFound);

    server.begin();
    serverStarted = true;
    LOG(LOG_INFO, "HTTP Routen konfiguriert und Server gestartet.");
}