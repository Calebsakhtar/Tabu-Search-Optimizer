#include <vector>
#include <string>
#include <math.h>

#include "../headers/STM.h"

namespace TS {

	// Although this implementation of TS is my own, the flow diagrams of the
	// following papers were followed closely:
	// 
	// S.Phillips and J.P.Jarrett, “Enhancing Designer Understanding by Combining
	// Multiple Dominance Relationsand Tabu Search,” in AIAA Scitech 2020 Forum.
	// doi:10.2514/6.2020-0161
	// 
	//
	// D. Jaeggi, G. Parks, T. Kipouros, and P. Clarkson, “A Multi-objective Tabu Search
	// Algorithm for Constrained Optimisation Problems, ” Lecture Notes in Computer
	//	Science, vol. 3410, pp. 490–504, Mar. 2005. doi:10.1007/978-3-540-31880-4_34.
	//

	// Intended constructor for the Variable class
	STM::STM(const size_t& STM_size, const Config& sample_config) {

		// Initialize the STM with the sample config
		for (size_t i = 0; i < STM_size; i++) {
			m_configs.push_back(sample_config);
		}

		m_size = STM_size;
	}

	// Verify whether a certain variable is in the STM
	bool STM::in_STM(const Config& candidate_config) const {

		// For each of the configurations in the STM
		for (size_t i = 0; i < m_configs.size(); i++) {

			// Initialize the bool to be returned. Initial assumption is true
			bool is_in_STM = true;

			// Get the current configuration
			Config current_config_STM = m_configs[i];

			// Check each variable in this configuration
			for (size_t j = 0; j < candidate_config.size_vars(); j++) {
				Variable current_var_STM = current_config_STM[j];
				Variable current_var_candidate = candidate_config[j];

				// If they are not the same variable, move to the next configuration
				if (!current_var_STM.same_variable_step(current_var_candidate)) {
					is_in_STM = false;
					break;
				}
			}

			// If we have found it is in STM, return true
			// (no need to check the other configurations)
			if (is_in_STM == true) {
				return true;
			}
			else {
				break;
			}
		}

		// Return false if no common configurations have been found
		return false;
	}

	// Replace the oldest configuration of the STM with the new point visited
	void STM::add_to_STM(const Config& new_point) {
		m_configs[m_index] = new_point;

		// Increase the index and use the modulo operator to make indexing circular
		m_index = (m_index + 1) % m_size;
	}

}
