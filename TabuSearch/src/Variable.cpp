#include <vector>
#include <string>
#include <math.h>

#include "../headers/Variable.h"

namespace TS {

	// Intended constructor for the Variable class
	Variable::Variable(const bool& discrete, const std::vector<std::array<double, 2>>& feasible_regions,
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
		bool same = true;

		// Get the step size of the input variable
		double stepsize = other.get_step();

		if (m_discrete != other.get_discrete()) { same = false; }
		if (m_name != other.get_name()) { same = false; }
		if (std::abs(m_val - other.get_val()) > 1e-6) { same = false; }

		// I did not check for the same feasible regions

		return same;
	};

	// Check whether two variables are the relative to the step size of the next variable
	bool Variable::same_variable_step(const Variable& other) const {

		// Keep a tally of how many things are the same
		bool same = true;

		// Get the step size of the input variable
		double stepsize = other.get_step();

		if (m_discrete != other.get_discrete()) { same = false; }
		if (m_name != other.get_name()) { same = false; }
		if (std::abs(m_val - other.get_val()) > stepsize / 4) { same = false; }

		// I did not check for the same feasible regions

		return same;
	};

}
