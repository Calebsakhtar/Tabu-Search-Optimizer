#include <vector>
#include <string>
#include <math.h>

#include "../headers/STM.h"

namespace TS {

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

		for (size_t i = 0; i < m_configs.size(); i++) {

			// Initialize the bool to be returned. Initial assumption is true
			bool is_in_STM = true;

			// Get the current configuration
			Config current_config_STM = m_configs[i];

			// Check each variable in this configuration
			for (size_t j = 1; j < candidate_config.size(); j++) {
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
