#ifndef TS_CONFIGURATION_H
#define TS_CONFIGURATION_H

#include <vector>
#include <string>

#include "Variable.h"
#include "../../MDR-Test/MDR Test Project/headers/DesignClasses.h"
#include "../../MDR-Test/MDR Test Project/headers/MDRFunctions.h"

namespace TS {

	// Although this implementation of TS is my own, the flow diagrams of the
	// following papers were used as a base for the implementation:
	// 
	// S.Phillips and J.P.Jarrett, ?Enhancing Designer Understanding by Combining
	// Multiple Dominance Relationsand Tabu Search,? in AIAA Scitech 2020 Forum.
	// doi:10.2514/6.2020-0161
	// 
	//
	// D. Jaeggi, G. Parks, T. Kipouros, and P. Clarkson, ?A Multi-objective Tabu Search
	// Algorithm for Constrained Optimisation Problems, ? Lecture Notes in Computer
	//	Science, vol. 3410, pp. 490?504, Mar. 2005. doi: 10.1007/978-3-540-31880-4_34.
	//

	class Config {
		std::vector<Variable> m_vars; // Stores the ranges of feasible region
		MDR::Design m_performances; // Store the performance metrics
												 // of the configuration
		bool m_perfs_evaluated = false; // Stores whether the performances have
										// been evaluated or not
		bool m_prev_move_increasing = true; // Store whether the previous move was an increase
		size_t m_prev_move_idx = 0; // Store the index of the previous move

	public:
		// Default constructor (constructs an empty object)
		Config() {};

		// Intended constructor
		Config(const std::vector<Variable>& vars) { m_vars = vars; };

		// Intended constructor (with performance metrics)
		Config(const std::vector<Variable>& vars,
			const MDR::Design& performances);

		// Copy constructor (Currently Disabled)
		//Config(const Config& ip_var) {};

		// Set the performance metrics
		void set_performances(const MDR::Design& performances);

		// Set the previous move data
		void set_prev_move_data(const bool& increasing, const size_t& move_idx);

		// Initialize the rank data
		void initialize_ranks(const std::vector<MDR::DomRel>& dom_rels);

		// Change the value of the selected variable by the stepsize
		void change_var(const size_t& idx, const bool& increase);

		// Perform a reduction move
		void reduce(const double& reduction_factor);

		// Copy the stepsized from another configuration
		void copy_stepsizes(const Config& ip_config);

		// Get the performance metrics
		MDR::Design get_performances() const { return m_performances; };

		// Get the ranks
		std::vector<size_t> get_ranks() const { return m_performances.get_ranks(); };

		// Get the variables
		std::vector<Variable> get_vars() const { return m_vars; };

		// Return the size of the variables
		size_t size_vars() const { return m_vars.size(); };

		// Get the previous move data
		void get_prev_move_data(bool& increasing, size_t& move_idx) const;

		// Return whether all variables are feasible or not
		bool is_feasible() const;

		// Return whether the minimum step size has been reached
		bool min_size_reached() const;

		// Overload the index operator
		Variable operator [](int i) const { return m_vars[i]; }
		Variable& operator [](int i) { return m_vars[i]; }

		//// Overload the index operator
		//Variable operator [](int i) const { return m_vars[i]; }
		//Variable& operator [](int i) { return m_vars[i]; }
	};

}

#endif
