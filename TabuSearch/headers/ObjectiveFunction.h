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

	// Given an input configuration, evaluate its performance and
	// update the configuration with the performance metrics
	void compute_f(TS::Config& ip_config);
}

#endif
