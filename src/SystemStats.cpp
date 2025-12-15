// src/SystemStats.cpp
#include "SystemStats.h"
#include "AppLogger.h"
#include <ESP.h>

// --- GLOBALE DEFINITIONEN ---
const int STATS_WINDOW_SIZE = 100;
const unsigned long MAX_LOOP_DURATION_US = 10000;
volatile unsigned long loop_durations_buffer[STATS_WINDOW_SIZE];
volatile int stats_index = 0;
volatile unsigned long max_loop_duration_us = 0;
volatile unsigned long avg_loop_duration_us = 0;
volatile bool is_critical_latency_active = false;
static int calc_counter = 0;

// --- MINIMALE FUNKTIONEN ---
void recordLoopDuration(unsigned long duration_us) { /* Platzhalter */ }
void calculateLoopStatistics() {}
void printLoopStatus() { LOG(LOG_DEBUG, "SystemStatus: OK."); }
void checkHeapHealth() { /* Platzhalter */ }