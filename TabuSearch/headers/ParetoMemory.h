#ifndef TS_PARETO_MEMORY_H
#define TS_PARETO_MEMORY_H

#include <vector>
#include <string>

#include "Config.h"
#include "../../MDR-Test/MDR Test Project/headers/DesignClasses.h"
#include "../../MDR-Test/MDR Test Project/headers/MDRFunctions.h"

namespace TS {

	class ParetoMemory {
		std::vector<Config> m_configs; // Stores the ranges of feasible region

	public:
		// Default constructor (constructs an empty object)
		ParetoMemory() {};

		// Intended constructor
		ParetoMemory(const Config& sample_config);

		// Copy constructor (Currently Disabled)
		//ParetoMemory(const ParetoMemory& ip_var) {};

		// Replace the oldest configuration of the ParetoMemory with the new point visited
		bool consider_config(const Config& new_config);
	};

}

#endif
