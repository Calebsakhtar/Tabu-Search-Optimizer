#include <vector>
#include <string>
#include <math.h>

#include "../headers/ParetoMemory.h"

namespace TS {

	// Intended constructor for the Variable class
	ParetoMemory::ParetoMemory(const Config& sample_config) {

		std::vector<Config> configs = { sample_config };

		m_configs = configs;
	}

	// Replace the oldest configuration of the ParetoMemory with the new point visited
	bool ParetoMemory::consider_config(const Config& new_config) {
		
		
		for (size_t i = 0; i < m_configs.size(); i++) {
			Config current_config = m_configs[i];
			MDR::Design current_design = current_config.get_performances();
			MDR::Design ip_design = new_config.get_performances();

			// Check that no members of the ParetoMemory dominate the new config
			if (MDR::A_dominates_B(current_design, ip_design)) {
				return false;
			}
			else if (MDR::A_dominates_B(ip_design, current_design)) {
				// If the input dominates any of the existing designs, remove the existing design
				m_configs.erase(m_configs.begin() + i);
			}
		}

		// Add the candidate design to the ParetoMemory
		m_configs.push_back(new_config);

		// The candidate configuration has been added to the object, so return true
		return true;
	}

}
