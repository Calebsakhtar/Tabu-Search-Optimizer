#ifndef TS_STM_H
#define TS_STM_H

#include <vector>
#include <string>

#include "Config.h"

namespace TS {

	class STM {
		std::vector<Config> m_configs; // Stores the ranges of feasible region
		size_t m_size = 0; // Stores the size of the STM
		size_t m_index = 0; // Stores the current index of the STM

	public:
		// Default constructor (constructs an empty object)
		STM() {};

		// Intended constructor
		STM(const size_t& STM_size, const Config& sample_config);

		// Copy constructor (Currently Disabled)
		//STM(const STM& ip_var) {};

		// Verify whether a certain variable is in the STM
		bool in_STM(const Config& candidate_config) const;

		// Replace the oldest configuration of the STM with the new point visited
		void add_to_STM(const Config& new_point);
	};

}

#endif
