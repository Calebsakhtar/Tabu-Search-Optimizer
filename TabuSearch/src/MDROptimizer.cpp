#include <vector>
#include <string>
#include <math.h>

#include "../headers/MDROptimizer.h"

namespace TS {

	// Intended constructor for the Variable class
	MDROptimizer::MDROptimizer(const std::vector<MDR::DomRel>& dom_rels, const size_t& STM_size,
		const Config& initial_config, const double reduction_factor, const unsigned seed,
		const size_t INTENSIFY, const size_t DIVERSIFY, const size_t REDUCE, 
		const size_t max_iter_num) {

		// Save the dominance relations and the initial configuration
		m_dom_rels = dom_rels;
		m_initial_config = initial_config;

		// Save the maximum allowed iterations
		m_max_iter_num = max_iter_num;

		// Save the counter milestones
		m_INTENSIFY = INTENSIFY;
		m_DIVERSIFY = DIVERSIFY;
		m_REDUCE = REDUCE;

		// Initialize the STM
		STM ip_STM(STM_size, initial_config);
		m_STM = ip_STM;

		// Initialize the MTM and the APM
		std::vector<Config> config_vect = { initial_config };
		ParetoMemory ip_MTM(config_vect, dom_rels);
		m_MTM = ip_MTM;
		m_APM = ip_MTM;

		// Initialize the IM
		std::vector<Config> config_vect2 = {};
		ParetoMemory ip_IM(config_vect, dom_rels);
		m_IM = ip_IM;

		// Initialize the LTM
		LTM ip_LTM(initial_config);
		m_LTM = ip_LTM;

		// Initialize the Random Number Generator
		std::default_random_engine generator(seed);
		m_generator = generator;
	}

	void MDROptimizer::perform_optimization() {

	
	}

}
