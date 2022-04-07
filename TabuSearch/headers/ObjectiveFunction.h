#ifndef TS_OBJECTIVE_FUNCT_H
#define TS_OBJECTIVE_FUNCT_H

#include <vector>
#include <string>
#include <math.h>

#include "Config.h"
#include "XPlaneConnect.h"
#include "../../H2-Aircraft-Model/H2-Aircraft-Model/headers/AircraftModel.h"
#include "../../H2-Aircraft-Model/H2-Aircraft-Model/headers/MathTools.h"
#include "../../H2-Aircraft-Model/H2-Aircraft-Model/headers/PlaneMakerTools.h"

namespace AircraftEval {

	// Initialize the xplane simulation
	void init_simulator(XPCSocket sock);

	// Given an input configuration, evaluate its performance and
	// update the configuration with the performance metrics
	bool compute_f(TS::Config& ip_config);
}

#endif
