#ifndef TS_VARIABLE_H
#define TS_VARIABLE_H

#include <vector>
#include <string>

namespace TS {

	class Variable {
		bool m_discrete = false; // Stores whether the variable is discrete or not
		std::vector<double[2]> m_feasible_regions; // Stores the ranges of feasible region 
		double m_scale = 1; // Stores the scale of the variable (i.e. the size of the minstep)
		double m_val = 0; // Value of the variable
		std::string	m_name; // Variable name

	public:
		// Default constructor (constructs an empty object)
		Variable() {};

		// Intended constructor
		Variable(const bool& discrete, const std::vector<double[2]>& feasible_regions,
			const double& scale, const double value, const std::string& name);

		// Copy constructor (Currently Disabled)
		//Variable(const Variable& ip_var) {};
	};

}

#endif
