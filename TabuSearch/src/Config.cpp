#include <vector>
#include <string>
#include <math.h>

#include "../headers/Config.h"

namespace TS {

	// Intended constructor for the Variable class
	Config::Config(const std::vector<Variable>& vars,
		const MDR::Design& performances) {
		m_vars = vars;
		m_performances = performances;
		m_perfs_evaluated = true;
	}

	void Config::set_performances(const MDR::Design& performances) {
		m_performances = performances;
		m_perfs_evaluated = true;
	}

	// Perform a reduction move
	void Config::reduce(const double& reduction_factor) {
		// Iterate at each variable
		for (size_t i = 0; i < m_vars.size(); i++) {
			// If the variable is not discrete, reduce the step
			if (!m_vars[i].get_discrete()) {
				double current_stepsize = m_vars[i].get_step();
				m_vars[i].set_step(reduction_factor * current_stepsize);
			}
		}
	}

	bool Config::is_feasible() const {
		
		// Placeholder for a required input
		size_t idx = 0;

		// Check that all variables are feasible
		for (size_t i = 0; i < m_vars.size(); i++) {

			// If any one variable is infeasible, return false
			if (m_vars[i].is_feasible(idx) == false) {
				return false;
			}
		}

		// If all variables are feasible, return true
		return true;
	}
}
