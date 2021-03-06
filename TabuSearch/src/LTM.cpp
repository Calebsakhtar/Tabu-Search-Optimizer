#include <vector>
#include <string>
#include <math.h>

#include "../headers/LTM.h"

namespace TS 
{

	// Although this implementation of TS is my own, the flow diagrams of the
	// following papers were followed closely:
	// 
	// S.Phillips and J.P.Jarrett, ?Enhancing Designer Understanding by Combining
	// Multiple Dominance Relationsand Tabu Search,? in AIAA Scitech 2020 Forum.
	// doi:10.2514/6.2020-0161
	// 
	//
	// D. Jaeggi, G. Parks, T. Kipouros, and P. Clarkson, ?A Multi-objective Tabu Search
	// Algorithm for Constrained Optimisation Problems, ? Lecture Notes in Computer
	//	Science, vol. 3410, pp. 490?504, Mar. 2005. doi:10.1007/978-3-540-31880-4_34.
	//

	// Intended constructor for the Variable class
	LTM::LTM(const Config& sample_config) {

		// Extract all variables from the sample config (contains data of the feasible region)
		std::vector<Variable> sample_vars =  sample_config.get_vars();
		
		// Set the value of all variables to zero for reference
		for (size_t i = 0; i < sample_vars.size(); i++) {
			sample_vars[i].set_val(0);
		}

		m_sample_vars = sample_vars;

		// Make a tally vector and reserve the size of the sample variable vector
		std::vector<std::vector<size_t>> tally(m_sample_vars.size());

		// Iterate for each variable
		for (size_t i = 0; i < m_sample_vars.size(); i++) {
			// Get the feasible regions for the current
			std::vector<std::array<double, 2>> current_feas_regs = m_sample_vars[i].get_feas_regs();
			
			// Store the size of the current variable tally
			size_t n = 0;

			// If there are less than 2 feasible regions, split the design space into 2
			if (current_feas_regs.size() < 2) {
				n = 2;
			}
			else {
				n = current_feas_regs.size();
			}
			
			// Make the tally for the current variable
			std::vector<size_t> current_var_tally(n,0);

			// Save to the overall tally
			tally[i] = current_var_tally;
		}

		m_tally = tally;
	}

	// Given an input configuration, update the tally of locations visited
	void LTM::update_tally(const Config& new_pt) {

		// Extract all variables from the new point
		std::vector<Variable> new_pt_vars = new_pt.get_vars();

		// For every variable
		for (size_t i = 0; i < m_sample_vars.size(); i++) {
			Variable current_var_new = new_pt_vars[i];
			size_t tally_idx = 0;

			// Check the number of feasible regions
			const auto current_var_new_frs = current_var_new.get_feas_regs();

			// If there is only one feasible region, the index is different
			if (current_var_new_frs.size() < 2) {
				// If there is only one feasible region, the space is divided into two
				if (current_var_new.get_val() >= (current_var_new_frs[0][1] + current_var_new_frs[0][0]) / 2) {
					tally_idx = 1;
				}
			
				// No need to assign an index for the FALSE case due to the initialization
			}
			else {
				// Check the variable value is feasible and get the index where it acts
				assert(current_var_new.is_feasible(tally_idx));
			}

			// Update the relevant section of the tally
			m_tally[i][tally_idx] += 1;

		}
	
	}

	// Choose a random within the least visited region so far.
	// The generator input is for the randomness aspect.
	void LTM::diversify(Config& ip_config, std::default_random_engine& generator){
		
		// Use the input variables as a base to make a new config
		std::vector<Variable> result_vars = ip_config.get_vars();

		// Increase the index and use the modulo operator to make indexing circular
		m_last_var_idx = (m_last_var_idx + 1) % m_sample_vars.size();
		
		size_t i = m_last_var_idx;

		// Recover the current variable and its feasible regions
		Variable current_var = result_vars[i];
		const auto current_var_frs = current_var.get_feas_regs();

		// Recover the index of the region with the minimum element
		std::vector<size_t> fr_tallies = m_tally[i];
		const auto min_iterator = std::min_element(fr_tallies.begin(), fr_tallies.end());
		size_t min_idx = std::distance(fr_tallies.begin(), min_iterator);

		// Initialize the ranges for the uniform distribution to generate the new config
		double range_start = 0;
		double range_end = 0;

		if (current_var_frs.size() < 2) {
			// If there is only one feasible region, the space is divided into two
			if (min_idx == 0) {
				range_start = current_var_frs[0][0];
				range_end = (current_var_frs[0][1] + current_var_frs[0][0]) / 2;
			}
			else {
				range_start = (current_var_frs[0][1] + current_var_frs[0][0]) / 2;
				range_end = current_var_frs[0][1];
			}
		}
		else {
			// Otherwise, treat the feasible regions as normal
			auto current_fr = current_var_frs[min_idx];
			range_start = current_fr[0];
			range_end = current_fr[1];
		}

		// Generate the value for the current variable 
		double gen_number = 0;

		// Account for discrete variables
		if (current_var.get_discrete()) {
			int range_start_int = static_cast<int>(floor(range_start));
			int range_end_int = static_cast<int>(ceil(range_end));
			std::uniform_int_distribution<> distribution(range_start_int, range_end_int);
			gen_number = static_cast<double>(distribution(generator));
		}
		else {
			// Generate the value for the current variable 
			std::uniform_real_distribution<double> distribution(range_start, range_end);
			gen_number = distribution(generator);
		}

		current_var.set_val(gen_number);

		result_vars[i] = current_var;

		// Make a new config to return
		Config result_config(result_vars);

		result_config.copy_stepsizes(ip_config);

		ip_config = result_config;
	};

}
