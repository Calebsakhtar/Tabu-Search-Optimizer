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
    // Initialize the vector containing the variables
    std::vector<TS::Variable> vars;

    // Create a variable for the Hydrogen power fraction
    std::array<double, 2> feas_reg_H2_Pfrac = { 0, 1 };
    std::vector<std::array<double, 2>> feas_regs_H2_Pfrac = { feas_reg_H2_Pfrac };
    const double start_H2_Pfrac = 0.0;
    const double stepsize_H2_Pfrac = 0.2;
    const double stepsize_min_H2_Pfrac = 0.001;
    const std::string name_H2_Pfrac = "Hydrogen Power Fraction";
    TS::Variable var_H2_Pfrac(false, feas_regs_H2_Pfrac, stepsize_min_H2_Pfrac, start_H2_Pfrac,
        stepsize_H2_Pfrac, name_H2_Pfrac);
    vars.push_back(var_H2_Pfrac);

    // Create a variable for the range
    std::array<double,2> feas_reg_range = { 500, 2800 };
    std::vector<std::array<double, 2>> feas_regs_range = { feas_reg_range };
    const double start_range = 1400; // km
    const double stepsize_range = 400; // km
    const double stepsize_min_range = 0.01; // km
    const std::string name_range = "Range (km)";
    TS::Variable var_range(false, feas_regs_range, stepsize_min_range, start_range, stepsize_range,
        name_range);
    vars.push_back(var_range);
    
    // Create a variable for the max. single engine power
    std::array<double, 2> feas_reg_Pmax = { 1500, 3500 }; //kW
    std::vector<std::array<double, 2>> feas_regs_Pmax = { feas_reg_Pmax };
    const double start_Pmax = 2500;
    const double stepsize_Pmax = 500;
    const double stepsize_min_Pmax = 50;
    const std::string name_Pmax = "Max. Single Engine Power (kW)";
    TS::Variable var_Pmax(false, feas_regs_Pmax, stepsize_min_Pmax, start_Pmax, stepsize_Pmax, 
        name_Pmax);
    vars.push_back(var_Pmax);

    // Create a variable for the Cruise Altitude
    std::array<double, 2> feas_reg_h = { 4.500, 7.620 }; // km (15000 - 25000 ft)
    std::vector<std::array<double, 2>> feas_regs_h = { feas_reg_h };
    const double start_h = 6.100; // km (20000 ft)
    const double stepsize_h = 0.620; // km
    const double stepsize_min_h = 0.100; // km
    const std::string name_h = "Cruise Altitude (km)";
    TS::Variable var_h(false, feas_regs_h, stepsize_min_h, start_h, stepsize_h, name_h);
    vars.push_back(var_h);

    // Create a variable for the Cruise Mach Number
    std::array<double, 2> feas_reg_M = { 0.4, 0.67 };
    std::vector<std::array<double, 2>> feas_regs_M = { feas_reg_M };
    const double start_M = 0.6;
    const double stepsize_M = 0.2;
    const double stepsize_min_M = 0.05;
    const std::string name_M = "Cruise Mach Number";
    TS::Variable var_M(false, feas_regs_M, stepsize_min_M, start_M, stepsize_M, name_M);
    vars.push_back(var_M);

    // Make the initial point from the variables
    TS::Config initial_point(vars);

    // Specify the MDR Layers of Dominance
    MDR::DomRel first_layer(0, 1);
    MDR::DomRel first_layer1(0, 2);
    std::vector<MDR::DomRel> dom_rels = { first_layer, first_layer1};

    //MDR::DomRel second_layer(2, 3);
    //MDR::DomRel third_layer(3, 4);
    //std::vector<MDR::DomRel> dom_rels = { first_layer, second_layer, third_layer };

    // Specify the TS Parameters
    size_t STM_size = 7;
    double reduction_factor = 0.95;
    unsigned seed = 1;
    size_t INTENSIFY = 10;
    size_t DIVERSIFY = 15;
    size_t REDUCE = 25;
    size_t max_eval_num = 20000;
    size_t HJ_num = 8;

    // Set up the simulation
    printf("Setting up Simulation\n");

    // Open Socket
    const char* IP = "192.168.1.150";     //IP Address of computer running X-Plane
    XPCSocket sock = openUDP(IP);
    float tVal[1];
    int tSize = 1;
    if (getDREF(sock, "sim/test/test_float", tVal, &tSize) < 0)
    {
        printf("Error establishing connecting. Unable to read data from X-Plane.");
        return EXIT_FAILURE;
    }
    else {
        printf("Initial connection successful.\n");
    }

    // Instantiate the Optimizer object
    TS::MDROptimizer Optimizer(dom_rels, STM_size, initial_point, reduction_factor, seed, sock, INTENSIFY,
        DIVERSIFY, REDUCE, max_eval_num, HJ_num);

    // Perform the optimization and store the results
    Optimizer.perform_optimization();
    auto result_MTM = Optimizer.retreive_MTM();
    auto all_pts = Optimizer.retrieve_all_pts();

    // Print the relevant quantities for MATLAB Visualization
    Optimizer.print_pareto_front();
    Optimizer.print_visited_pts();
    Optimizer.print_visited_pts_coords();

    // Mark the end of the program
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
