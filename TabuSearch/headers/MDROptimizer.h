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
#include "../../MDR-Test/MDR Test Project/headers/DesignClasses.h"
#include "../../MDR-Test/MDR Test Project/headers/MDRFunctions.h"

namespace TS {

	class MDROptimizer {
		std::vector<MDR::DomRel> m_dom_rels; // Stores the dominance relations
		Config m_initial_config; // Stores a sample config
		std::default_random_engine m_generator; // Stores the RN Generator
		
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
			const Config& sample_config, const unsigned seed);

		// Copy constructor (Currently Disabled)
		//Optimizer(const Optimizer& ip_var) {};

	};

}

#endif
