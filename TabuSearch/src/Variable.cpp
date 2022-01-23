#include <vector>
#include <string>
#include <math.h>

#include "../headers/Variable.h"

namespace TS {

	// Intended constructor for the Variable class
	Variable::Variable(const bool& discrete, const std::vector<std::vector<double>>& feasible_regions,
		const double& scale, const double& value, const double& stepsize, 
		const std::string& name) {

		m_discrete = discrete;
		m_feasible_regions = feasible_regions;
		m_scale = scale;
		m_val = value;
		m_step = stepsize;
		m_name = name;
	}

	// Check whether two variables are the same
	bool Variable::same_variable(const Variable& other) const {

		// Keep a tally of how many things are the same
		int counter = 0;

		if (m_discrete == other.get_discrete()) { counter++; }
		if (m_name == other.get_name()) { counter++; }
		if (std::abs(m_val - other.get_val()) < 1e-6) { counter++; }

		// I did not check for the same feasible regions

		if (counter > 2) {
			return true;
		}
		return false;
	};

	// Check whether two variables are the relative to the step size of the next variable
	bool Variable::same_variable_step(const Variable& other) const {

		// Keep a tally of how many things are the same
		int counter = 0;

		// 
		double stepsize = other.get_step();

		if (m_discrete == other.get_discrete()) { counter++; }
		if (m_name == other.get_name()) { counter++; }
		if (std::abs(m_val - other.get_val()) < stepsize / 4) { counter++; }

		// I did not check for the same feasible regions

		if (counter > 2) {
			return true;
		}
		return false;
	};

}
