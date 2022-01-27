#ifndef TS_IM_H
#define TS_IM_H

#include <vector>
#include <string>

#include "Config.h"
#include "../../MDR-Test/MDR Test Project/headers/DesignClasses.h"
#include "../../MDR-Test/MDR Test Project/headers/MDRFunctions.h"

namespace TS {

	class IM {
		std::vector<Config> m_configs; // Stores the ranges of feasible region

	public:
		// Default constructor (constructs an empty object)
		IM() {};

		// Intended constructor
		IM(const Config& sample_config);

		// Copy constructor (Currently Disabled)
		//IM(const IM& ip_var) {};

		// Replace the oldest configuration of the IM with the new point visited
		bool consider_config(const Config& new_config);
	};

}

#endif
