#include <vector>
#include <string>
#include <math.h>

#include "../headers/MDROptimizer.h"

namespace TS {

	// Intended constructor for the Variable class
	MDROptimizer::MDROptimizer(const std::vector<MDR::DomRel>& dom_rels, const size_t& STM_size,
		const Config& initial_config, const double reduction_factor, const unsigned seed,
		const size_t INTENSIFY, const size_t DIVERSIFY, const size_t REDUCE, 
		const size_t max_eval_num, const size_t HJ_num) {

		// Save the dominance relations and the initial configuration
		m_dom_rels = dom_rels;
		m_initial_config = initial_config;

		// Compute the objective function (result stored in candidate_point)
		AircraftEval::compute_f(m_initial_config);
		m_initial_config.initialize_ranks(m_dom_rels);
		m_f_eval_num = 1;

		// Save the maximum allowed iterations
		m_f_eval_num_max = max_eval_num;

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
		std::vector<Config> config_vect = { m_initial_config };
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

		// Set the current point and the next as the initial point
		Config current_config = m_initial_config;
		Config next_config = m_initial_config;

		// Store the number of variables
		size_t num_vars = current_config.get_vars().size();

		// Store the previous move data
		bool prev_increase = true;
		size_t prev_idx = 1e10;
		
		// Main loop begin (stop when either the minimum size is reached or the
		// maximum iteration number is reached)
		while (!current_config.min_size_reached() && (m_f_eval_num < m_f_eval_num_max)) {
			// Increase the iteration number
			m_iter_num++;
			
			// Initialize a vector to store all the H&J points
			std::vector<Config> HJ_configs = {};

			// Generate the candidate points using Hooke & Jeeves moves
			for (size_t i = 0; i < num_vars; i++) {

				Config HJ_config = current_config;

				// Check whether this was the previous move to avoid unnecessary function evaluations
				if (!((prev_idx == i) && (prev_increase = true))) {
					// Increase the variable
					HJ_config.change_var(i, true);

					if (HJ_config.is_feasible() && !m_STM.in_STM(HJ_config)) {
						// Only add the point to the list of candidate points if it is feasible
						// and not Tabu.
						HJ_configs.push_back(HJ_config);
					}
				}

				// Check whether this was the previous move to avoid unnecessary function evaluations
				if (!((prev_idx == i) && (prev_increase = false))) {
					// Decrease the variable
					HJ_config = current_config;
					HJ_config.change_var(i, false);

					if (HJ_config.is_feasible() && !m_STM.in_STM(HJ_config)) {
						// Only add the point to the list of candidate points if it is feasible
						// and not Tabu.
						HJ_configs.push_back(HJ_config);
					}
				}
			}

			// Initialise the dominant_configs vector
			std::vector<Config> dominant_configs = {};
			
			// Initialise move_on
			bool move_on = false;

			// Initialise whether a dominant point has been found
			bool dominant_found = false;

			// If all new points are either Tabu or Unfeasible, diversify
			if (HJ_configs.size() < 1) {
				current_config = m_LTM.diversify(m_generator);

				// Compute the objective function
				AircraftEval::compute_f(current_config);
				m_f_eval_num++;
				current_config.initialize_ranks(m_dom_rels);

				// Add the current point to the MTM, STM and LTM
				m_MTM.consider_config(current_config);
				m_STM.add_to_STM(current_config);
				m_LTM.update_tally(current_config);

				// Add the canidate points to the All Point Memory (APM) and update its rank
				m_APM.add_config_update_ranks(current_config);

				// Move to the next iteration
				break;
			}


			// Loop to figure out the best generated point
			while (!move_on) {

				// Initialize a vector to store all the candidate points
				std::vector<Config> candidate_configs = {};

				// If there are more points than the maximum to be considered, choose the points
				// to be considered at random by shuffling the H&J vector with the seeded generator
				if (HJ_configs.size() > m_HJ_num) {
					std::shuffle(HJ_configs.begin(), HJ_configs.end(), m_generator);
				}

				// Make a new HJ config vector
				std::vector<Config> HJ_configs_new = HJ_configs;

				// Include the first m_HJ_num points to be considered
				for (size_t i = 0; i < m_HJ_num; i++) {
					// Recover the point to be considered
					Config candidate_config = HJ_configs[i];

					// Erase the config from the new HJ config_vector
					HJ_configs_new.erase(HJ_configs_new.begin() + i);

					// Compute the objective function (result stored in candidate_point)
					AircraftEval::compute_f(candidate_config);
					m_f_eval_num++;
					candidate_config.initialize_ranks(m_dom_rels);

					// Store the point in the set of candidate points
					candidate_configs.push_back(candidate_config);

					// Add the canidate points to the All Point Memory (APM) and update its rank
					m_APM.add_config_update_ranks(candidate_config);

					// Protect against going out of index range
					if (i + 1 >= HJ_configs.size()) {
						break;
					}
				}

				// Remove the designs that have been chosen
				HJ_configs = HJ_configs_new;

				// ParetoMemory has the functionality to update the rankings as required
				std::vector<Config> first_config = { candidate_configs[0] };
				ParetoMemory candidate_config_memory(first_config, m_dom_rels);

				// Remove any non-dominant designs
				for (size_t i = 1; i < candidate_configs.size(); i++) {
					candidate_config_memory.consider_config_MDR(candidate_configs[i]);
				}

				// Recover the dominant configurations
				dominant_configs = candidate_config_memory.get_configs();

				// Shuffle the vector of dominant configurations if necessary
				if (dominant_configs.size() > 1) {
					std::shuffle(dominant_configs.begin(), dominant_configs.end(), m_generator);
				}

				// Bool to move on
				move_on = false;

				// Check if any design dominates the current design
				for (size_t i = 0; i < dominant_configs.size(); i++) {
					if (MDR::A_dominates_B_MDR(dominant_configs[i].get_performances(),
						current_config.get_performances(), m_dom_rels)) {
						
						// We have found the new point
						move_on = true;
						next_config = dominant_configs[i];

						// This new point is dominant
						dominant_found = true;

						// No need to check any other points
						break;
					}
				}

				// If no designs dominate the current design, check to see there are some designs
				// to consider
				if (move_on == false) {
					if (HJ_configs.size() < 1) {
						// If there are no alternative designs, choose a point at random
						move_on = true;
						next_config = dominant_configs[0];
					}
				}
			}

			// Update the current point
			current_config = next_config;

			// Update the move data
			current_config.get_prev_move_data(prev_increase, prev_idx);
			
			// Add the current point to the MTM, STM and LTM
			bool new_MTM_config = m_MTM.consider_config(current_config);
			m_STM.add_to_STM(current_config);
			m_LTM.update_tally(current_config);

			// Add the remaining points to the IM
			for (size_t i = 1; i < dominant_configs.size(); i++) {
				m_IM.consider_config(dominant_configs[i]);
			}

			// Attempt a pattern move
			if (dominant_found) {
				next_config = current_config;

				// Perform the same move as the previous move
				next_config.change_var(prev_idx, prev_increase);

				// Compute the objective function (result stored in candidate_point)
				AircraftEval::compute_f(next_config);
				m_f_eval_num++;
				next_config.initialize_ranks(m_dom_rels);

				// Check whether this move dominates the current point
				if (MDR::A_dominates_B_MDR(next_config.get_performances(),
					current_config.get_performances(), m_dom_rels)) {

					// Update the current point
					current_config = next_config;

					// Update the move data
					current_config.get_prev_move_data(prev_increase, prev_idx);

					// Add the current point to the MTM, STM and LTM
					new_MTM_config = new_MTM_config || m_MTM.consider_config(current_config);
					m_STM.add_to_STM(current_config);
					m_LTM.update_tally(current_config);

					// Add the canidate points to the All Point Memory (APM) and update its rank
					m_APM.add_config_update_ranks(current_config);
				}
			}

			// Check to see that the maximum number of evaluations has been reached
			if (m_f_eval_num > m_f_eval_num_max) {
				break;
			}

			// Reset or increase the counter as needed
			if (new_MTM_config) {
				m_counter = 0;
			}
			else {
				m_counter++;
			}

			// Intensify, Diversify or Reduce as appropriate
			if (m_counter == m_INTENSIFY){ 
				current_config = m_IM.intensify(m_generator);

				// Compute the objective function
				AircraftEval::compute_f(current_config);
				m_f_eval_num++;
				current_config.initialize_ranks(m_dom_rels);

				// Add the current point to the MTM, STM and LTM
				m_MTM.consider_config(current_config);
				m_STM.add_to_STM(current_config);
				m_LTM.update_tally(current_config);

				// Add the canidate points to the All Point Memory (APM) and update its rank
				m_APM.add_config_update_ranks(current_config);
			
			}
			else if (m_counter == m_DIVERSIFY) {
				current_config = m_LTM.diversify(m_generator);

				// Compute the objective function
				AircraftEval::compute_f(current_config);
				m_f_eval_num++;
				current_config.initialize_ranks(m_dom_rels);

				// Add the current point to the MTM, STM and LTM
				m_MTM.consider_config(current_config);
				m_STM.add_to_STM(current_config);
				m_LTM.update_tally(current_config);

				// Add the canidate points to the All Point Memory (APM) and update its rank
				m_APM.add_config_update_ranks(current_config);

			}
			else if (m_counter == m_REDUCE) {
				current_config.reduce(m_reduction_factor);
			}

		}

	
	}

}
