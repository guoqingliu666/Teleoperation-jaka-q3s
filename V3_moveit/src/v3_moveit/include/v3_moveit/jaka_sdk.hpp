#pragma once

// The bundled SDK exports C symbols, but jakaAPI.h checks the misspelled
// __cpluscplus macro. Keep the vendor files unchanged and set linkage here.
extern "C" {
#include "jakaAPI.h"
}
