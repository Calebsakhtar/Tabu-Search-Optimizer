#ifndef TS_VARIABLE_H
#define TS_VARIABLE_H

#include <vector>
#include <string>

namespace TS {

	class Variable {
		bool m_discrete = false; // Stores whether the variable is discrete or not
		std::vector<std::vector<double>> m_feasible_regions; // Stores the ranges of feasible region 
		double m_scale = 1; // Stores the scale of the variable (i.e. the size of the minstep)
		double m_step = 0; // Value of the current stepsize
		double m_val = 0; // Value of the variable
		std::string	m_name = "N/A"; // Variable name

	public:
		// Default constructor (constructs an empty object)
		Variable() {};

		// Intended constructor
		Variable(const bool& discrete, const std::vector<std::vector<double>>& feasible_regions,
			const double& scale, const double& value, const double& stepsize,
			const std::string& name);

		// Copy constructor (Currently Disabled)
		//Variable(const Variable& ip_var) {};

		// Set the variable scale
		void set_scale(const double& scale) { m_scale = scale; };

		// Get the variable scale
		double get_scale() const { return m_scale; };

		// Set the variable step size
		void set_step(const double& stepsize) { m_step = stepsize; };

		// Get the variable step size
		double get_step() const { return m_step; };

		// Set the variable value
		void set_val(const double& val) { m_val = val; };

		// Get the variable value
		double get_val() const { return m_val; };

		// Get the variable name
		std::string get_name() const { return m_name; };

		// Get the feasible regions
		std::vector<std::vector<double>> get_feas_regs() const { return m_feasible_regions; };

		// Say whether the variable is discrete or not
		bool get_discrete() const { return m_discrete; };

		// Check whether two variables are the same
		bool same_variable(const Variable& other) const;

		// Check whether two variables are the relative to the step size of the next variable
		bool same_variable_step(const Variable& other) const;

		// Overload the comparison operators
		// (Code obtained from: https://stackoverflow.com/questions/4421706/what-are-the-basic-rules-and-idioms-for-operator-overloading)
		friend bool operator == (const Variable& lhs, const Variable& rhs) {
			return lhs.get_val() == rhs.get_val(); };
		friend bool operator != (const Variable& lhs, const Variable& rhs) {
			return !operator == (lhs, rhs); };
		friend bool operator < (const Variable& lhs, const Variable& rhs) {
			return lhs.get_val() < rhs.get_val();};
		friend bool operator > (const Variable& lhs, const Variable& rhs) {
			return  operator < (rhs, lhs); };
		friend bool operator <= (const Variable& lhs, const Variable& rhs) {
			return !operator > (lhs, rhs); };
		friend bool operator >= (const Variable& lhs, const Variable& rhs) {
			return !operator < (lhs, rhs); };
	};

}

#endif
