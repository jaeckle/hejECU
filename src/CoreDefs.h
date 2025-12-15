// src/CoreDefs.h
#pragma once

// =========================================================
// 1. Logging und Basistypen
// =========================================================

// Definition der Log-Level
typedef enum {
    LOG_NONE = 0,
    LOG_ERROR,
    LOG_WARN,
    LOG_INFO,
    LOG_DEBUG
} LogLevel;

// WICHTIG: ALLE STRUCT DEFINITIONEN WURDEN NACH IGNITIONMANAGER.H VERSCHOBEN!
// (z.B. IgnitionState, SpeedState, RpmConfig, IgnitionMap2D)