#include "global.h"

// raw readings
float g_dhtTemp[3]     = {0};
float g_dhtHum[3]      = {0};

// instantaneous AH
float g_dhtAH[3]       = {0};

// diffs vs. sensor0
float g_dhtAHDiff[2]   = {0};

// EMAs (seed to NAN so first sample initializes)
float g_dhtAH_ema[3]       = { NAN, NAN, NAN };
float g_dhtAHDiff_ema[2]   = { NAN, NAN };

// status strings
char g_dhtStatus[2][5]     = {"dry", "dry"};

    