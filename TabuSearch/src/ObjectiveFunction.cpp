#include <vector>
#include <string>
#include <math.h>

#include "../headers/ObjectiveFunction.h"

namespace AircraftEval {

	// Given an input configuration, evaluate its performance and
	// update the configuration with the performance metrics
	void compute_f(TS::Config& ip_config) {

		// THIS IS A PLACEHOLDER FUNCTION

		std::vector<TS::Variable> variables = ip_config.get_vars();

		double obj_A = 0;
		double obj_B = 0;

		for (size_t i = 0; i < variables.size(); i++) {
			double var_val = variables[i].get_val();
			obj_A += (var_val - 4) * var_val + 3;
			obj_B += 3*sin(var_val*10);//+= 5 - var_val;
		}

		// Formally Store Performance Metric A
		MDR::MetricID idA("Metric A", 0);
		MDR::PerfMetric perfA(idA, obj_A, true);

		// Formally Store Performance Metric B
		MDR::MetricID idB("Metric B", 1);
		MDR::PerfMetric perfB(idB, obj_B/3, true);

		// Put them in a vector
		std::vector<MDR::PerfMetric> perf_vect = { perfA, perfB };

		// This is a bit silly but necessary
		size_t zero = 0;
		size_t one = 1;

		// Make a Design object and assign it to the input configuration
		MDR::Design performances(perf_vect, zero, zero, one);
		ip_config.set_performances(performances);
	}

}
