#ifndef TS_LTM_H
#define TS_LTM_H

#include <vector>
#include <string>
#include <random>

#include "Variable.h"
#include "Config.h"

namespace TS {

	// Although this implementation of TS is my own, the flow diagrams of the
	// following papers were followed closely:
	// 
	// S.Phillips and J.P.Jarrett, “Enhancing Designer Understanding by Combining
	// Multiple Dominance Relationsand Tabu Search,” in AIAA Scitech 2020 Forum.
	// doi:10.2514/6.2020-0161
	// 
	//
	// D. Jaeggi, G. Parks, T. Kipouros, and P. Clarkson, “A Multi-objective Tabu Search
	// Algorithm for Constrained Optimisation Problems, ” Lecture Notes in Computer
	//	Science, vol. 3410, pp. 490–504, Mar. 2005. doi:10.1007/978-3-540-31880-4_34.
	//

	class LTM {
		std::vector<Variable> m_sample_vars; // Stores the ranges of feasible region
		std::vector<std::vector<size_t>> m_tally; // Stores the tally of the LTM
		size_t m_last_var_idx = 0; // Stores the index of the last variable changed

	public:
		// Default constructor (constructs an empty object)
		LTM() {};

		// Intended constructor
		LTM(const Config& sample_config);

		// Given an input configuration, update the tally of locations visited
		void update_tally(const Config& new_pt);

		// Choose a random within the least visited region so far
		void diversify(Config& ip_config, std::default_random_engine& generator);
	};
}

#endif
