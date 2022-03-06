// TabuSearch.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <array>
#include <fstream>

#include "headers/MDR-TS.h"
#include "headers/XPlaneConnect.h"

int main()
{
    // Create two variables
    std::array<double,2> feasible_region = { 0, 1 };
    std::vector<std::array<double, 2>> feasible_regions = { feasible_region };
    TS::Variable var1(false, feasible_regions, 1, 10, 1, "a");
    TS::Variable var2(false, feasible_regions, 1, 9, 1, "b");

    // Test whether the "<" operator works as expected
    if (!(var1 < var2)) {
        std::cout << "The < operator works!\n\n";
    }

    // Make and test the configuration
    TS::Config config({ var1, var2 });
    if (config[1] == var2) {
        std::cout << "The configuration works!\n\n";
    }

    // Test the STM
    TS::STM STM(3, config);
    TS::Config config2({ var2, var1 });

    if (!STM.in_STM(config2)) {
        std::cout << "The in_STM method works!\n\n";
    }
    
    // Test the Addition of new elements to the STM
    STM.add_to_STM(config2);

    if (STM.in_STM(config2)) {
        std::cout << "The add_to_STM method works!\n\n";
    }

    // Test the circular nature of the STM
    for (size_t i = 0; i < 7; i++) {
        STM.add_to_STM(config2);
    }

    if (!STM.in_STM(config)) {
        std::cout << "The STM is circular!\n\n";
    }

    // MAIN OPTIMIZATION TEST
    
    // Specify the required parameters
    MDR::DomRel dom_rel(0, 1);
    std::vector<MDR::DomRel> dom_rels = { dom_rel };
    size_t STM_size = 7;
    double reduction_factor = 0.5;
    unsigned seed = 1;
    size_t INTENSIFY = 10;
    size_t DIVERSIFY = 15;
    size_t REDUCE = 25;
    size_t max_eval_num = 2000;
    size_t HJ_num = 8;

    // Initialize the variables and the vector containing them
    std::array<double, 2> feas_reg_low1 = { 0., 0.45 };
    std::array<double, 2> feas_reg_high1 = { 0.77, 1. };
    std::vector<std::array<double, 2>> feas_regs1 = {feas_reg_low1, feas_reg_high1};
    std::array<double, 2> feas_reg_a = { 0.4, 0.7 };
    std::array<double, 2> feas_reg_b = { 0.9, 1. };
    std::vector<std::array<double, 2>> feas_regs2 = { feas_reg_a, feas_reg_b };
    TS::Variable varA(false, feas_regs1, 0.02, 0.2, 0.2, "Variable A");
    TS::Variable varB(false, feas_regs2, 0.02, 0.5, 0.2, "Variable B");
    std::vector<TS::Variable> vars = { varA, varB };

    TS::Config initial_point(vars);

    TS::MDROptimizer Optimizer(dom_rels, STM_size, initial_point, reduction_factor, seed, INTENSIFY,
        DIVERSIFY, REDUCE, max_eval_num, HJ_num);

    //Optimizer.perform_optimization();

    //auto result_MTM = Optimizer.retreive_MTM();
    //auto all_pts = Optimizer.retrieve_all_pts();

    //// Print the relevant quantities for MATLAB Visualization
    //Optimizer.print_pareto_front();
    //Optimizer.print_visited_pts();
    //Optimizer.print_visited_pts_coords();

    // Finish the test
    std::cout << "Hello World!\n";

	printf("XPlaneConnect Example Script\n- Setting up Simulation\n");

	// Open Socket
	const char* IP = "128.232.250.212";     //IP Address of computer running X-Plane
	XPCSocket sock = openUDP(IP);
	float tVal[1];
	int tSize = 1;
	if (getDREF(sock, "sim/test/test_float", tVal, &tSize) < 0)
	{
		printf("Error establishing connecting. Unable to read data from X-Plane.");
		return EXIT_FAILURE;
	}

	// Set Location/Orientation (sendPOSI)
	// Set Up Position Array
	double POSI[7] = { 0.0 };

    POSI[0] = 51.875278627882849;
    POSI[1] = 0.22022808392539564;
    POSI[2] = 6395.0574894603342;
    POSI[3] = -1.6440951824188232;
    POSI[4] = -0.14142291247844696;
    POSI[5] = 42.833587646484375;
    POSI[6] = 0.0;

    sendPOSI(sock, POSI, 7, 0);

    const char* speed_dref = "sim/flightmodel/position/local_vx"; // real DREF
    float speed_val = 143.377; 
    sendDREF(sock, speed_dref, &speed_val, 1); // Send data

    const char* ap_off_dref = "sim/cockpit/autopilot/autopilot_mode"; // AP Mode
    float ap_off_val = 0; // off=0, flight director=1, on=2
    sendDREF(sock, ap_off_dref, &ap_off_val, 1); // Send data

    const char* ap_state_dref = "sim/cockpit/autopilot/autopilot_state"; // AP State

    // pauseSim
    pauseSim(sock, 1); // Sending 1 to pause	
    sleep(5); // Pause for 5 seconds

    const char* ap_mode_dref = "sim/cockpit/autopilot/autopilot_mode"; // AP Mode
    float ap_mode_val = 2; // off=0, flight director=1, on=2
    sendDREF(sock, ap_mode_dref, &ap_mode_val, 1); // Send data

    const char* ap_alt_dref = "sim/cockpit/autopilot/altitude"; // AP Altitude
    float ap_alt_val = 21000; // in ft above sea level
    sendDREF(sock, ap_alt_dref, &ap_alt_val, 1); // Send data

    const char* ap_vs_dref = "sim/cockpit/autopilot/vertical_velocity"; // AP Vertical Speed
    float ap_vs_val = 1000; // in ft/min above sea level
    sendDREF(sock, ap_vs_dref, &ap_vs_val, 1); // Send data

    const char* ap_hdg_dref = "sim/cockpit/autopilot/heading"; // AP Heading
    float ap_hdg_val = 43; // in ft/min above sea level
    sendDREF(sock, ap_hdg_dref, &ap_hdg_val, 1); // Send data

    float ap_vnav_val = 131072; // arm vnav
    sendDREF(sock, ap_state_dref, &ap_vnav_val, 1); // Send data
    
    float ap_hnav_val = 256; // arm hnav
    sendDREF(sock, ap_state_dref, &ap_hnav_val, 1); // Send data

    float ap_athr_val = 1; // change autothrottle
    sendDREF(sock, ap_state_dref, &ap_athr_val, 1); // Send data

    float ap_hdg2_val = 2; // hdg hold
    sendDREF(sock, ap_state_dref, &ap_hdg2_val, 1); // Send data

    //float ap_vsset_val = 16; // change vspeed
    //sendDREF(sock, ap_state_dref, &ap_vsset_val, 1); // Send data

    //float ap_flchange_val = 64; // change flight level
    //sendDREF(sock, ap_state_dref, &ap_flchange_val, 1); // Send data


    //POSI[3] = 180; // Pitch
    //POSI[4] = 90; // Roll (+ve to the right)
    //sendPOSI(sock, POSI, 7, 0);

	// pauseSim
	pauseSim(sock, 1); // Sending 1 to pause	
	sleep(5); // Pause for 5 seconds

	// Unpause
	pauseSim(sock, 0); // Sending 0 to unpause
	printf("- Resuming Simulation\n");

	// Simulate for 10 seconds
	sleep(10);

	printf("---End Program---\n");

	return 0;

}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
