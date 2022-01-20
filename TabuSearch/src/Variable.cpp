#include <vector>
#include <string>
#include <math.h>

#include "../headers/Variable.h"

namespace TS {

	// Intended constructor
	Variable::Variable(const bool& discrete, const std::vector<double[2]>& feasible_regions,
		const double& scale, const double value, const std::string& name) {

		m_discrete = discrete;
		m_feasible_regions = feasible_regions;
		m_scale = scale;
		m_val = value;
		m_name = name;
	}

}
