// src/SystemStats.h
#pragma once
#include <Arduino.h>

// --- STATISTIK & RESSOURCEN (Konstanten/Externs) ---
extern const int STATS_WINDOW_SIZE;
extern const unsigned long MAX_LOOP_DURATION_US;

// Globale Variablen (Definition in SystemStats.cpp)
extern volatile unsigned long loop_durations_buffer[];
extern volatile int stats_index;
extern volatile unsigned long max_loop_duration_us;
extern volatile unsigned long avg_loop_duration_us;
extern volatile bool is_critical_latency_active;

// Funktionen
void calculateLoopStatistics();
void printLoopStatus();
void recordLoopDuration(unsigned long duration_us);
void checkHeapHealth();