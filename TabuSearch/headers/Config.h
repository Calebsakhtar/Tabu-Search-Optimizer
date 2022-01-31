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
		MDR::Design m_performances; // Store the performance metrics
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
			const MDR::Design& performances);

		// Copy constructor (Currently Disabled)
		//Config(const Config& ip_var) {};

		// Set the performance metrics
		void set_performances(const MDR::Design& performances);

		// Perform a reduction move
		void reduce(const double& step_size);

		// Get the performance metrics
		MDR::Design get_performances() const { return m_performances; };

		// Get the ranks
		std::vector<size_t> get_ranks() const { return m_performances.get_ranks(); };

		// Get the variables
		std::vector<Variable> get_vars() const { return m_vars; };

		// Return the size of the variables
		size_t size_vars() const { return m_vars.size(); };

		// Return whether all variables are feasible or not
		bool is_feasible() const;

		// Return whether the minimum step size has been reached
		bool min_size_reached() const;

		// Overload the index operator
		Variable operator [](int i) const { return m_vars[i]; }
		Variable& operator [](int i) { return m_vars[i]; }

		//// Overload the index operator
		//Variable operator [](int i) const { return m_vars[i]; }
		//Variable& operator [](int i) { return m_vars[i]; }
	};

}

#endif
