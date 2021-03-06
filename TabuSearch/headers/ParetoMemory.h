#ifndef TS_PARETO_MEMORY_H
#define TS_PARETO_MEMORY_H

#include <vector>
#include <string>
#include <random>

#include "Config.h"
#include "../../MDR-Test/MDR Test Project/headers/DesignClasses.h"
#include "../../MDR-Test/MDR Test Project/headers/MDRFunctions.h"

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

	class ParetoMemory {
		std::vector<Config> m_configs; // Stores the ranges of feasible region
		std::vector<MDR::DomRel> m_dom_rels; // Stores the dominance relations

	public:
		// Default constructor (constructs an empty object)
		ParetoMemory() {};

		// Intended constructor
		ParetoMemory(const std::vector<Config>& configs,
			const std::vector<MDR::DomRel>& dom_rels);

		// Copy constructor (Currently Disabled)
		//ParetoMemory(const ParetoMemory& ip_var) {};

		// Get a list of the performances
		std::vector<MDR::Design> get_perf_vect() const;

		// Return the current configuration vector
		std::vector<Config> get_configs() const { return m_configs; };

		// Set the performances from an input vector of performances
		void set_perf_vect(const std::vector<MDR::Design>& ip_vect);

		//// DEPRECATED
		//// Replace the oldest configuration of the ParetoMemory with the new point visited
		//bool consider_config(const Config& new_config);

		// Replace the oldest configuration of the ParetoMemory with the new point visited
		// (using MDR)
		bool consider_config_MDR(const Config& new_config);

		// Add a new configuration to the pareto memory and update the ranks of every config
		void add_config_update_ranks(const Config& new_config);

		// Perform an intesnification move (TO BE USED IN THE IM)
		void intensify(Config& config_to_change, std::default_random_engine& generator);

		// Overload the index operator
		Config operator [](int i) const { return m_configs[i]; };
		Config& operator [](int i) { return m_configs[i]; };
	};

}

#endif
