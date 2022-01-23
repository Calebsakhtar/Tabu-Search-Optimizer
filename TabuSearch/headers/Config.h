#ifndef TS_CONFIGURATION_H
#define TS_CONFIGURATION_H

#include <vector>
#include <string>

#include "Variable.h"
#include "../../MDR-Test/MDR Test Project/headers/DesignClasses.h"
#include "../../MDR-Test/MDR Test Project/headers/MDRFunctions.h"

namespace TS {

	//using Config = std::vector<Variable>;

	class Config {
		std::vector<Variable> m_vars; // Stores the ranges of feasible region
		std::vector<MDR::Design> m_performances; // Store the performance metrics
												 // of the configuration
		bool m_perfs_evaluated = false; // Stores whether the performances have
										// been evaluated or not

	public:
		// Default constructor (constructs an empty object)
		Config() {};

		// Intended constructor
		Config(const std::vector<Variable>& vars) { m_vars = vars; };

		// Intended constructor (with performance metrics)
		Config(const std::vector<Variable>& vars,
			const std::vector<MDR::Design>& performances) {
			m_vars = vars;
			m_performances = performances;
		};

		// Copy constructor (Currently Disabled)
		//Config(const Config& ip_var) {};

		// Set the performance metrics
		void set_performances(const std::vector<MDR::Design>& performances) {
			m_performances = performances;
		};

		// Return the size of the variables
		size_t size_vars() const { return m_vars.size(); };

		// Overload the index operator
		Variable operator [](int i) const { return m_vars[i]; }
		Variable& operator [](int i) { return m_vars[i]; }

		// Make a new index operator for performances
		MDR::Design operator ()(int i) const { return m_performances[i]; }
		MDR::Design& operator ()(int i) { return m_performances[i]; }
	};

}

#endif
