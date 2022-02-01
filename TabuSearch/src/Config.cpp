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
