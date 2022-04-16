#ifndef TS_OPTIMIZER_H
#define TS_OPTIMIZER_H

#include <vector>
#include <string>
#include <random>
#include <fstream>

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
		size_t m_f_eval_num = 0; // Stores the number of times the objective function is called
		size_t m_iter_num = 0; // Stores the iteration number
		size_t m_f_eval_num_max = 1e10; // Stores the maximum iteration number
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

		// Store the XPC Socket
		XPCSocket m_sock;

		// Store whether the optimization has run or not
		bool m_optimized = false;

		// Find all layers of the pareto front
		bool find_pareto_front_layers(std::vector<std::vector<Config>>& 
			op_vect) const;

		// Print the performances of all visited points
		bool print_configs(const std::vector<TS::Config>& configs, 
			const std::string& filename) const;

	public:
		// Default constructor (constructs an empty object)
		MDROptimizer() {};

		// Intended constructor
		MDROptimizer(const std::vector<MDR::DomRel>& dom_rels, const size_t& STM_size,
			const Config& initial_config, const double reduction_factor, const unsigned seed, 
			const XPCSocket sock, const size_t INTENSIFY = 10, const size_t DIVERSIFY = 15, 
			const size_t REDUCE = 25, const size_t max_eval_num = 1e100, const size_t HJ_num = 8);

		// Copy constructor (Currently Disabled)
		//Optimizer(const Optimizer& ip_var) {};

		// Perform the optimization routine
		void perform_optimization();

		// Retreive the optimization results
		std::vector<Config> retreive_MTM() const { return m_MTM.get_configs(); };

		// Retrieve all visited points
		std::vector<Config> retrieve_all_pts() const { return m_APM.get_configs(); };

		// Print the performances of all visited points
		bool print_visited_pts_loc_perf() const;

		// Print the performances of all pareto front points
		bool print_pareto_front() const;

		// Print all of the pareto front layers
		bool print_pareto_front_layers() const;

	};


}

#endif
