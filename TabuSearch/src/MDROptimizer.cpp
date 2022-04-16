#include <vector>
#include <string>
#include <math.h>

#include "../headers/MDROptimizer.h"

namespace TS {


	// PRIVATE FUNCTIONS
		

	
	// Print the performances
	bool MDROptimizer::print_configs(const std::vector<TS::Config>& configs, 
		const std::string& filename) const {

		// Check input
		if (configs.size() < 1) {
			// Invalid input so return false
			return false;
		}



		// PRINT THE PERFORMANCES 
		


		// Create the Performances File Location
		std::string perf_fileloc = "Results/" + filename + "Perf.csv";

		// Create and open a text file to store the performances of all the Visited Points
		std::ofstream OpFilePerf(perf_fileloc);

		size_t perf_size2 =
			configs[0].get_performances().get_perf_vector().size();

		OpFilePerf << "PointID";

		for (size_t i = 0; i < perf_size2; i++) {
			OpFilePerf << ",Objective" << std::to_string(i + 1);
		}

		OpFilePerf << "\n";

		for (size_t i = 0; i < configs.size(); i++) {
			std::vector<MDR::PerfMetric> current_perfs2 =
				configs[i].get_performances().get_perf_vector();

			OpFilePerf << std::to_string(i + 1) << ",";

			for (size_t j = 0; j < current_perfs2.size() - 1; j++) {
				OpFilePerf << std::to_string(current_perfs2[j].get_metric_val()) << ",";
			}

			OpFilePerf << std::to_string(current_perfs2[current_perfs2.size() - 1].get_metric_val()) << "\n";
		}

		// Close the file
		OpFilePerf.close();



		// PRINT THE POINT LOCATIONS



		// Create the Performances File Location
		std::string locs_fileloc = "Results/" + filename + "Loc.csv";

		// Create and open a text file to store the Coordinates of all the Visited Points
		std::ofstream OpFileLoc(locs_fileloc);

		auto vars_size =
			configs[0].get_vars().size();

		OpFileLoc << "PointID";

		for (size_t i = 0; i < vars_size; i++) {
			OpFileLoc << ",Variable" << std::to_string(i + 1);
		}

		OpFileLoc << "\n";

		for (size_t i = 0; i < configs.size(); i++) {
			std::vector<TS::Variable> current_vars =
				configs[i].get_vars();

			OpFileLoc << std::to_string(i + 1) << ",";

			for (size_t j = 0; j < current_vars.size() - 1; j++) {
				OpFileLoc << std::to_string(current_vars[j].get_val()) << ",";
			}

			OpFileLoc << std::to_string(current_vars[current_vars.size() - 1].get_val()) << "\n";
		}

		// Close the file
		OpFileLoc.close();

		// Return true (the operation was successful)
		return true;
	}



	// PUBLIC FUNCTIONS



	// Intended constructor for the Variable class
	MDROptimizer::MDROptimizer(const std::vector<MDR::DomRel>& dom_rels, const size_t& STM_size,
		const Config& initial_config, const double reduction_factor, const unsigned seed,
		const XPCSocket sock, const size_t INTENSIFY, const size_t DIVERSIFY,
		const size_t REDUCE, const size_t max_eval_num, const size_t HJ_num) {

		// Save the XPC Socket
		m_sock = sock;

		// Save the dominance relations and the initial configuration
		m_dom_rels = dom_rels;
		m_initial_config = initial_config;

		// Compute the objective function (result stored in candidate_point)
		AircraftEval::compute_f(m_initial_config, m_sock, m_f_eval_num);
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

		// Initialize the Simulation
		AircraftEval::init_simulator(m_sock);

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

			// If all new points are either Tabu or Unfeasible, diversify and reduce if necessary
			if (HJ_configs.size() < 1) {
				m_LTM.diversify(current_config, m_generator);

				// Compute the objective function
				AircraftEval::compute_f(current_config, m_sock, m_f_eval_num);
				m_f_eval_num++;
				current_config.initialize_ranks(m_dom_rels);

				// Add the current point to the MTM, STM and LTM
				m_MTM.consider_config_MDR(current_config);
				m_STM.add_to_STM(current_config);
				m_LTM.update_tally(current_config);

				// Add the canidate points to the All Point Memory (APM) and update its rank
				m_APM.add_config_update_ranks(current_config);

				// Increase the counter
				m_counter++;

				// Reduce if necessary
				if (m_counter == m_REDUCE) {
					current_config.reduce(m_reduction_factor);
					m_counter = 0;
				}

				// Move to the next iteration
				continue;
			}

			// Store whether a new configuration has been added to the MTM
			bool new_MTM_config;

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
					HJ_configs_new.erase(HJ_configs_new.begin());

					// Compute the objective function (result stored in candidate_point)
					AircraftEval::compute_f(candidate_config, m_sock, m_f_eval_num);
					m_f_eval_num++;
					candidate_config.initialize_ranks(m_dom_rels);

					// Store the point in the set of candidate points
					candidate_configs.push_back(candidate_config);

					// Add the canidate points to the All Point Memory (APM) and update its rank
					m_APM.add_config_update_ranks(candidate_config);

					// Add the candidate point to the MTM if necessary
					new_MTM_config = m_MTM.consider_config_MDR(candidate_config);

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
			
			// Add the current point to the STM
			m_STM.add_to_STM(current_config);
			m_LTM.update_tally(current_config);

			// Add the remaining points to the IM
			for (size_t i = 1; i < dominant_configs.size(); i++) {
				m_IM.consider_config_MDR(dominant_configs[i]);
			}

			// Attempt a pattern move
			if (dominant_found) {
				next_config = current_config;

				// Perform the same move as the previous move
				next_config.change_var(prev_idx, prev_increase);

				// Check whether this move is feasible and not tabu
				if (next_config.is_feasible() && !m_STM.in_STM(next_config)) {

					// Compute the objective function (result stored in candidate_point)
					AircraftEval::compute_f(next_config, m_sock, m_f_eval_num);
					m_f_eval_num++;
					next_config.initialize_ranks(m_dom_rels);
					
					// Add the canidate points to the All Point Memory (APM) and update its rank
					m_APM.add_config_update_ranks(next_config);

					// Add the point to the MTM
					new_MTM_config = new_MTM_config || m_MTM.consider_config_MDR(current_config);

					// Check whether this move dominates the current point
					if (MDR::A_dominates_B_MDR(next_config.get_performances(),
						current_config.get_performances(), m_dom_rels)) {

						// Update the current point
						current_config = next_config;

						// Update the move data
						current_config.get_prev_move_data(prev_increase, prev_idx);

						// Add the current point to the STM and LTM
						m_STM.add_to_STM(current_config);
						m_LTM.update_tally(current_config);
					}
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
				m_IM.intensify(current_config, m_generator);

				// Compute the objective function
				AircraftEval::compute_f(current_config, m_sock, m_f_eval_num);
				m_f_eval_num++;
				current_config.initialize_ranks(m_dom_rels);

				// Add the current point to the MTM, STM and LTM
				m_MTM.consider_config_MDR(current_config);
				m_STM.add_to_STM(current_config);
				m_LTM.update_tally(current_config);

				// Add the canidate points to the All Point Memory (APM) and update its rank
				m_APM.add_config_update_ranks(current_config);
			
			}
			else if (m_counter == m_DIVERSIFY) {
				m_LTM.diversify(current_config, m_generator);

				// Compute the objective function
				AircraftEval::compute_f(current_config, m_sock, m_f_eval_num);
				m_f_eval_num++;
				current_config.initialize_ranks(m_dom_rels);

				// Add the current point to the MTM, STM and LTM
				m_MTM.consider_config_MDR(current_config);
				m_STM.add_to_STM(current_config);
				m_LTM.update_tally(current_config);

				// Add the canidate points to the All Point Memory (APM) and update its rank
				m_APM.add_config_update_ranks(current_config);

			}
			else if (m_counter == m_REDUCE) {
				current_config.reduce(m_reduction_factor);
				m_counter = 0;
			}

		}
		
		// When the optimization has finished, save the state
		m_optimized = true;
	}

	// Find all pareto front layers
	bool MDROptimizer::find_pareto_front_layers(std::vector<std::vector<Config>>&
		op_vect) const {

		// If not optimized, fail
		if (!m_optimized) {
			return false;
		}

		// Retrieve all visited points
		std::vector<TS::Config> current_pts = retrieve_all_pts();

		// If the current points size is zero, or there are no dom rels, fail
		if (current_pts.size() < 1 || m_dom_rels.size() < 1) {
			return false;
		}

		// Initialize a temporary output
		std::vector<std::vector<Config>> paretos_list(m_dom_rels.size());

		// Define the maximum rank
		size_t max_rank = current_pts.size() + 1;

		// For each dominance relation, we need to find the pareto front
		for (size_t i = 0; i < m_dom_rels.size(); i++) {

			// Initialize a temp vector
			std::vector<TS::Config> current_pareto;

			// Initialize the minimum known rank
			size_t min_rank = max_rank;

			// For each member of the current point vector, check whether its rank
			// is below the suspected minimum
			for (size_t j = 0; j < current_pts.size(); j++) {

				Config current_pt = current_pts[j];
				size_t current_rank = current_pt.get_ranks()[i];

				if (current_rank < min_rank) {
					// If the current rank is beneath the known min rank set the new min rank.
					min_rank = current_rank;
				}
			}

			// For each member of the current point vector, check whether its rank
			// is equal to the known minimum
			for (size_t j = 0; j < current_pts.size(); j++) {

				Config current_pt = current_pts[j];
				size_t current_rank = current_pt.get_ranks()[i];

				// Add the point to the pareto front
				if (current_rank == min_rank) {
					current_pareto.push_back(current_pt);
				}
			}

			// Set the found pareto front as the ith OP pareto front
			paretos_list[i] = current_pareto;

			// If there is only one point left, populate the rest of the output vector
			// with the one point and stop
			if (current_pareto.size() < 2) {

				for (size_t k = i + 1; k < m_dom_rels.size(); k++) {
					paretos_list[k] = current_pareto;
				}

				// Output the pareto fronts
				op_vect = paretos_list;
				return true;
			}

			// The next set of candidate points are the current pareto front points
			current_pts = current_pareto;
		}

		// Output the pareto fronts
		op_vect = paretos_list;
		return true;
	}

	// Print the performances and locations of all visited points
	bool MDROptimizer::print_visited_pts_loc_perf() const {

		if (m_optimized) {
			// Retrieve all visited points
			std::vector<TS::Config> all_pts = retrieve_all_pts();

			// Create and open a text file to store the performances of all the Visited Points
			std::string filename = "AllPoints";

			return print_configs(all_pts, filename);

		}
		else
		{
			return false;
		}

	}

	// Print the performances of all pareto front points
	bool MDROptimizer::print_pareto_front() const {

		if (m_optimized) {
			// Recover the pareto front
			std::vector<TS::Config> result_MTM = retreive_MTM();

			// Create and open a text file to store the performances of all the Visited Points
			std::string filename = "ParetoFrontFinal";

			return print_configs(result_MTM, filename);

		}
		else
		{
			return false;
		}
	}

	bool MDROptimizer::print_pareto_front_layers() const {

		// Check that the optimization has been performed
		if (!m_optimized) {
			return false;
		}

		// Retrieve all visited points
		std::vector<std::vector<TS::Config>> pareto_fronts;

		// Check retrieval of pareto front is successful
		if (!find_pareto_front_layers(pareto_fronts)) {
			return false;
		}

		// Check that the pareto fronts are not empty
		if (pareto_fronts.size() < 1) {
			return false;
		}

		// Initialize the Filename Base
		std::string filename_base = "ParetoFrontLayer";

		// Initialize the print success bool
		bool print_success = true;

		// For each pareto front
		for (size_t i = 0; i < pareto_fronts.size(); i++) {
			
			// Initialize the filename
			std::string filename = filename_base;

			// Add the correct number
			if (i < 10) {
				filename += "00" + std::to_string(i);
			}
			else {
				filename += "0" + std::to_string(i);
			}

			// Print the current pareto front
			print_success &= print_configs(pareto_fronts[i], filename);
		}

		return print_success;
	}
}
