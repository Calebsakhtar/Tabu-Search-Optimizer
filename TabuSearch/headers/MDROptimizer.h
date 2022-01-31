#ifndef TS_OPTIMIZER_H
#define TS_OPTIMIZER_H

#include <vector>
#include <string>
#include <random>

#include "Variable.h"
#include "Config.h"
#include "STM.h"
#include "ParetoMemory.h"
#include "LTM.h"
#include "ObjectiveFunction.h"
#include "../../MDR-Test/MDR Test Project/headers/DesignClasses.h"
#include "../../MDR-Test/MDR Test Project/headers/MDRFunctions.h"

namespace TS {

	class MDROptimizer {
		std::vector<MDR::DomRel> m_dom_rels; // Stores the dominance relations
		Config m_initial_config; // Stores a sample config
		std::default_random_engine m_generator; // Stores the RN Generator
		size_t m_iter_num = 0; // Stores the iteration number
		size_t m_max_iter_num = 1e100; // Stores the maximum iteration number
		size_t m_HJ_num = 8; // Stores the maximum number of H&J points considered
		double m_reduction_factor = 0.5; // Stores the minimum reduction factor

		// Store the counter milestones
		size_t m_counter = 0;
		size_t m_INTENSIFY = 10;
		size_t m_DIVERSIFY = 15;
		size_t m_REDUCE = 25;

		// Store the different data memories
		STM m_STM; // Short-Term-Memory
		ParetoMemory m_MTM; // Medium-Term-Memory
		ParetoMemory m_IM; // Intensification Memory
		ParetoMemory m_APM; // All-Point Memory
		LTM m_LTM; // Long-Term Memory

	public:
		// Default constructor (constructs an empty object)
		MDROptimizer() {};

		// Intended constructor
		MDROptimizer(const std::vector<MDR::DomRel>& dom_rels, const size_t& STM_size,
			const Config& initial_config, const double reduction_factor, const unsigned seed, 
			const size_t INTENSIFY = 10, const size_t DIVERSIFY = 15, const size_t REDUCE = 25, 
			const size_t max_iter_num = 1e100, const size_t HJ_num = 8);

		// Copy constructor (Currently Disabled)
		//Optimizer(const Optimizer& ip_var) {};

		// Perform the optimization routine
		void perform_optimization();
	};


}

#endif
