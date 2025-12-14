// WebServer.h

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <ESP8266WebServer.h>
#include "CoreDefs.h" 

// --- HANDLER DEKLARATIONEN ---
void handleRoot();
void handleNotFound();
void handleRestart();

// Log-Handler
void handleViewLog();
void handleSetLogLevel();
void handleClearLog();
// logCriticalEvent ist jetzt in AppLogger.h deklariert

// Konfigurations-Handler
void handleConfig();
void handleSaveConfig();
void handleRpmConfig();
void handleSaveRpmConfig();
void handleRestoreCurve();

// 2D Map Handler (NEU)
void handleMapEditor();         
void handleMapApiData();        
void handleMapApiUpdate();      
String generateMapEditorHtml(); 

// System/Setup
void setup_server_routes();
void setup_ota();
void registerMapRoutes(); // <--- NEU: Deklariert die ausgelagerte Funktion
bool isAuthorized(); // Externe Deklaration


#endif // WEBSERVER_H