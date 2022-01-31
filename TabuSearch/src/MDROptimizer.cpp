#include <vector>
#include <string>
#include <math.h>

#include "../headers/MDROptimizer.h"

namespace TS {

	// Intended constructor for the Variable class
	MDROptimizer::MDROptimizer(const std::vector<MDR::DomRel>& dom_rels, const size_t& STM_size,
		const Config& initial_config, const double reduction_factor, const unsigned seed,
		const size_t INTENSIFY, const size_t DIVERSIFY, const size_t REDUCE, 
		const size_t max_iter_num, const size_t HJ_num) {

		// Save the dominance relations and the initial configuration
		m_dom_rels = dom_rels;
		m_initial_config = initial_config;

		// Save the maximum allowed iterations
		m_max_iter_num = max_iter_num;

		// Save the maximum number of H&J points allowed
		m_HJ_num = HJ_num;

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
		
		// Set the current point as the initial point
		Config current_point = m_initial_config;

		// Store the number of variables
		size_t num_vars = current_point.get_vars().size();
		
		// Main loop begin (stop when either the minimum size is reached or the
		// maximum iteration number is reached)
		while (!current_point.min_size_reached() && (m_iter_num < m_max_iter_num)) {
			// Increase the iteration number
			m_iter_num++;
			
			// Initialize a vector to store all the H&J points
			std::vector<Config> HJ_points = {};

			// Generate the candidate points using Hooke & Jeeves moves
			for (size_t i = 0; i < num_vars; i++) {

				// Increase the variable
				Config HJ_point = current_point;
				HJ_point.change_var(i, true);

				
				if (HJ_point.is_feasible()) {
					// Only add the point to the list of candidate points if it is feasible
					HJ_points.push_back(HJ_point);
				}

				// Decrease the variable
				HJ_point = current_point;
				HJ_point.change_var(i, false);

				if (HJ_point.is_feasible()) {
					// Only add the point to the list of candidate points if it is feasible
					HJ_points.push_back(HJ_point);
				}
			}

			// Initialize a vector to store all the candidate points
			std::vector<Config> candidate_points = {};

			// If there are more points than the maximum to be considered, choose the points
			// to be considered at random by shuffling the H&J vector with the seeded generator
			if (HJ_points.size() > m_HJ_num) {
				std::shuffle(HJ_points.begin(), HJ_points.end(), m_generator);
			}

			// Include the first m_HJ_num points to be considered
			for (size_t i = 0; i < m_HJ_num; i++) {
				// Recover the point to be considered
				Config candidate_point = HJ_points[i];

				// Compute the objective function (result stored in candidate_point)
				AircraftEval::compute_f(candidate_point);

				// Store the point in the set of candidate points
				candidate_points.push_back(candidate_point);

				// Protect against going out of index range
				if (i + 1 >= HJ_points.size()) {
					break;
				}
			}

			// NEXT BIT OF CODE GOES HERE

		}
	
	}

}
