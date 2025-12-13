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

#ifndef WEBSERVER_H
#define WEBSERVER_H

#include <ESP8266WebServer.h>
#include "CoreDefs.h" 

// Deklarationen aus main.cpp (für externe Verwendung)
extern ESP8266WebServer server;
extern volatile LogLevel activeLogLevel;
extern String apPassword; // Wird für isAuthorized benötigt
extern bool startApMode; 
extern const char *LOG_FILENAME;
extern std::vector<String> startupLogBuffer;
extern bool serverStarted;

// Externe Funktionen (aus main.cpp)
extern bool saveConfig();
extern void app_log(LogLevel level, const char* file, int line, int logLine, const char* format, ...);

// Log-Makro-Deklaration für WebServer.cpp
#define LOG_WEB(level, format, ...) app_log(level, "WebServer.cpp", __LINE__, __LINE__, format, ##__VA_ARGS__) 

// --- Funktions-Deklarationen ---
void setup_server_routes();
void setup_ota();
bool isAuthorized();

// Die kritische Log-Funktion, die in LittleFS speichert
void logCriticalEvent(unsigned long maxDuration, unsigned long timestamp, const char* status);

// Handler
void handleRoot();
void handleRestart();
void handleNotFound();

void handleViewLog(); 
void handleClearLog(); 
void handleSetLogLevel(); // NEU: Kombinierter POST-Handler für Log-Level

#endif // WEBSERVER_H