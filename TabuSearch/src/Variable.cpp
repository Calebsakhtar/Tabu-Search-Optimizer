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

}
