#ifndef TS_OBJECTIVE_FUNCT_H
#define TS_OBJECTIVE_FUNCT_H

#include <vector>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>

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
	bool compute_f(TS::Config& ip_config, const XPCSocket sock, const size_t num_f_evals = 0);

	// Given an input configuration, evaluate its performance and
	// update the configuration with the performance metrics
	// No simulation is used
	bool compute_f_nosim(TS::Config& ip_config, const size_t num_f_evals);

	// Variation of compute_f that allows off-design aircraft performance to be evaluated
	bool evaluate_off_design_aircraft(TS::Config& ip_config, const XPCSocket sock, const size_t num_f_evals,
		const double& desired_range, const double& desired_h, const double& desired_m);
}

#endif
