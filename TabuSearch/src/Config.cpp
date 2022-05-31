#include <vector>
#include <string>
#include <math.h>

#include "../headers/Config.h"

namespace TS {

	// Although this implementation of TS is my own, the flow diagrams of the
	// following papers were used as a base for the implementation:
	// 
	// S.Phillips and J.P.Jarrett, “Enhancing Designer Understanding by Combining
	// Multiple Dominance Relationsand Tabu Search,” in AIAA Scitech 2020 Forum.
	// doi:10.2514/6.2020-0161
	// 
	//
	// D. Jaeggi, G. Parks, T. Kipouros, and P. Clarkson, “A Multi-objective Tabu Search
	// Algorithm for Constrained Optimisation Problems, ” Lecture Notes in Computer
	//	Science, vol. 3410, pp. 490–504, Mar. 2005. doi: 10.1007/978-3-540-31880-4_34.
	//

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

	// Set the previous move data
	void Config::set_prev_move_data(const bool& increasing, const size_t& move_idx) {
		m_prev_move_increasing = increasing;
		m_prev_move_idx = move_idx;
	}

	// Initialize the rank data
	void Config::initialize_ranks(const std::vector<MDR::DomRel>& dom_rels) {
		std::vector<size_t> ranks(dom_rels.size(), 0);
		MDR::Design perfs = get_performances();
		perfs.set_ranks(ranks);
		set_performances(perfs);
	}

	// Change the value of the selected variable by the stepsize
	void Config::change_var(const size_t& idx, const bool& increase) {
		// Get the stepsize
		double step_size = m_vars[idx].get_step();
		double value = m_vars[idx].get_val();

		// Increase/Decrease the variable value, as required
		if (increase) {
			m_vars[idx].set_val(value + step_size);
		}
		else {
			m_vars[idx].set_val(value - step_size);
		}

		// Store the details of this move
		m_prev_move_idx = idx;
		m_prev_move_increasing = increase;
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

	// Copy the stepsized from another configuration
	void Config::copy_stepsizes(const Config& ip_config) {
		
		// Retrieve the input variables and check the input
		const std::vector<Variable> ip_vars = ip_config.get_vars();
		assert(ip_vars.size() == m_vars.size());

		// Ensure that the variables are the same and set the steps
		for (size_t i = 0; i < m_vars.size(); i++) {
			assert(ip_vars[i].get_name() == m_vars[i].get_name());
			m_vars[i].set_step(ip_vars[i].get_step());
		}
	}

	// Set the previous move data
	void Config::get_prev_move_data(bool& increasing, size_t& move_idx) const {
		increasing = m_prev_move_increasing;
		move_idx = m_prev_move_idx;
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

	// Return whether the minimum step size has been reached
	bool Config::min_size_reached() const {

		// Check each variable
		for (size_t i = 0; i < m_vars.size(); i++) {
			// If the variable is not discrete, check the stepsize
			if (!m_vars[i].get_discrete()) {
				double current_stepsize = m_vars[i].get_step();

				// Check if the current step size is smaller than the minimum allowed
				if (current_stepsize > m_vars[i].get_scale()) {
					return false;
				}
			}
		}

		// All variables must have converged for this to return true
		return true;

	}
}
