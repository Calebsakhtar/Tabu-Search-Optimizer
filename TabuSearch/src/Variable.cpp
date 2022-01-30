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

	// Check whether a value lies within the feasible region
	bool Variable::is_feasible(size_t& feasible_idx) const {

		// Check whether the value lies within any of the feasible regions
		for (size_t i = 0; i < m_feasible_regions.size(); i++) {
			std::array<double, 2> current_region = m_feasible_regions[i];

			if ((current_region[0] <= m_val) && (current_region[1] >= m_val)) {
				// Give the feasible region index
				feasible_idx = i;

				// If the value is within a feasible region, return true
				return true;
			}
		}

		// If the value was in no feasible region, return false
		return false;
	}

}
