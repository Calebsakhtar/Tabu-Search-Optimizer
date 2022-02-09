#ifndef TS_LTM_H
#define TS_LTM_H

#include <vector>
#include <string>
#include <random>

#include "Variable.h"
#include "Config.h"

namespace TS {

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
