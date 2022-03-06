#ifndef TS_OBJECTIVE_FUNCT_H
#define TS_OBJECTIVE_FUNCT_H

#include <vector>
#include <string>

#include "Config.h"
#include "XPlaneConnect.h"

namespace AircraftEval {

	// Given an input configuration, evaluate its performance and
	// update the configuration with the performance metrics
	void compute_f(TS::Config& ip_config);
}

#endif
