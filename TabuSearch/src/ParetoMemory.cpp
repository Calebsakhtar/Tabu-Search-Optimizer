#include <vector>
#include <string>
#include <math.h>

#include "../headers/ParetoMemory.h"

namespace TS {

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
	ParetoMemory::ParetoMemory(const std::vector<Config>& configs,
		const std::vector<MDR::DomRel>& dom_rels) {

		m_configs = configs;
		m_dom_rels = dom_rels;
	}

	// Get a list of the performances
	std::vector<MDR::Design> ParetoMemory::get_perf_vect() const {
		std::vector<MDR::Design> design_vect;

		for (size_t i = 0; i < m_configs.size(); i++) {
			design_vect.push_back(m_configs[i].get_performances());
		}

		return design_vect;
	}

	// Set the performances from an input vector of performances
	void ParetoMemory::set_perf_vect(const std::vector<MDR::Design>& ip_vect) {
		
		// Ensure the vector is the right length to protect against indexing out of range
		assert(m_configs.size() == ip_vect.size());

		for (size_t i = 0; i < m_configs.size(); i++) {
			m_configs[i].set_performances(ip_vect[i]);
		}
	}

	//// DEPRECATED
	//// Replace the oldest configuration of the ParetoMemory with the new point visited if this
	//// new point dominates the configurations. Idea to erase in opposite order from:
	//// https://stackoverflow.com/questions/3487717/erasing-multiple-objects-from-a-stdvector
	//bool ParetoMemory::consider_config(const Config& new_config) {

	//	MDR::Design ip_design = new_config.get_performances();
	//	std::vector<size_t> idx_to_erase;

	//	for (size_t i = 0; i < m_configs.size(); i++) {
	//		Config current_config = m_configs[i];
	//		MDR::Design current_design = current_config.get_performances();

	//		// Check that no members of the ParetoMemory dominate the new config
	//		if (MDR::A_dominates_B_2D(current_design, ip_design)) {
	//			return false;
	//		}
	//		else if (MDR::A_dominates_B_2D(ip_design, current_design)) {
	//			// If the input dominates any of the existing designs, remove the existing design
	//			idx_to_erase.push_back(i);
	//		}
	//	}

	//	// Remove the required indexes in reverse order to preserve correct elements
	//	if (idx_to_erase.size() > 0) {
	//		for (size_t i = 0; i < idx_to_erase.size(); i++) {

	//			size_t erase_idx = idx_to_erase[idx_to_erase.size() - i - 1];
	//			m_configs.erase(m_configs.begin() + erase_idx);
	//		}
	//	}
	//	else {
	//		return false;
	//	}

	//	// Add the candidate design to the ParetoMemory
	//	m_configs.push_back(new_config);

	//	// The candidate configuration has been added to the object, so return true
	//	return true;
	//}

	// Replace the oldest configuration of the ParetoMemory with the new point visited if this
	// new point dominates the configurations according to MDR. Idea to erase in opposite order from:
	// https://stackoverflow.com/questions/3487717/erasing-multiple-objects-from-a-stdvector
	bool ParetoMemory::consider_config_MDR(const Config& new_config) {

		MDR::Design ip_design = new_config.get_performances();
		std::vector<size_t> idx_to_erase;

		for (size_t i = 0; i < m_configs.size(); i++) {
			Config current_config = m_configs[i];
			MDR::Design current_design = current_config.get_performances();

			// Check that no members of the ParetoMemory dominate the new config
			if (MDR::A_dominates_B_MDR(current_design, ip_design, m_dom_rels)) {
				return false;
			}
			else if (MDR::A_dominates_B_MDR(ip_design, current_design, m_dom_rels)) {
				// If the input dominates any of the existing designs, remove the existing design
				idx_to_erase.push_back(i);
			}
		}

		// Remove the required indexes in reverse order to preserve correct elements
		if (idx_to_erase.size() > 0) {
			for (size_t i = 0; i < idx_to_erase.size(); i++) {
				size_t erase_idx = idx_to_erase[idx_to_erase.size() - i - 1];
				m_configs.erase(m_configs.begin() + erase_idx);
			}
		}

		// Add the candidate design to the ParetoMemory
		m_configs.push_back(new_config);

		// The candidate configuration has been added to the object, so return true
		return true;
	}

	void ParetoMemory::add_config_update_ranks(const Config& new_config) {
		
		// Get the relevant design performances
		MDR::Design new_design = new_config.get_performances();
		std::vector<MDR::Design> existing_designs = get_perf_vect();

		// Update the ranks of the existing designs due to the new design
		MDR::update_ranks(new_design, existing_designs, m_dom_rels);

		// Add the new design to the existing designs and update the ParetoMemory
		existing_designs.push_back(new_design);

		m_configs.push_back(new_config);
		set_perf_vect(existing_designs);
	}

	// Perform an intesnification move (TO BE USED IN THE IM)
	void ParetoMemory::intensify(Config& config_to_change, std::default_random_engine& generator) {
		
		// Shuffle the points vector https://stackoverflow.com/a/36922712
		std::shuffle(m_configs.begin(), m_configs.end(), generator);

		// Get the chosen config
		Config chosen_config = m_configs[0];

		// Copy the step sizes
		chosen_config.copy_stepsizes(config_to_change);

		// Change the input
		config_to_change = chosen_config;
		
	}
}
