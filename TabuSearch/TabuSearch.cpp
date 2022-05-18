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

// Print the performances
bool printconfigs(const std::vector<TS::Config>& configs, const std::string& filename) {

    // Check input
    if (configs.size() < 1) {
        // Invalid input so return false
        return false;
    }



    // PRINT THE PERFORMANCES 



    // Create the Performances File Location
    std::string perf_fileloc = "Results/" + filename + "Performance.csv";

    // Create and open a text file to store the performances of all the Visited Points
    std::ofstream OpFilePerf(perf_fileloc);

    size_t perf_size2 =
        configs[0].get_performances().get_perf_vector().size();

    OpFilePerf << "PointID";

    for (size_t i = 0; i < perf_size2; i++) {
        OpFilePerf << ",Objective" << std::to_string(i);
    }

    OpFilePerf << "\n";

    for (size_t i = 0; i < configs.size(); i++) {
        std::vector<MDR::PerfMetric> current_perfs2 =
            configs[i].get_performances().get_perf_vector();

        OpFilePerf << std::to_string(i) << ",";

        for (size_t j = 0; j < current_perfs2.size() - 1; j++) {
            OpFilePerf << std::to_string(current_perfs2[j].get_metric_val()) << ",";
        }

        OpFilePerf << std::to_string(current_perfs2[current_perfs2.size() - 1].get_metric_val()) << "\n";
    }

    // Close the file
    OpFilePerf.close();



    // PRINT THE POINT LOCATIONS



    // Create the Performances File Location
    std::string locs_fileloc = "Results/" + filename + "Vector.csv";

    // Create and open a text file to store the Coordinates of all the Visited Points
    std::ofstream OpFileLoc(locs_fileloc);

    auto vars_size =
        configs[0].get_vars().size();

    OpFileLoc << "PointID";

    for (size_t i = 0; i < vars_size; i++) {
        OpFileLoc << ",Variable" << std::to_string(i);
    }

    OpFileLoc << "\n";

    for (size_t i = 0; i < configs.size(); i++) {
        std::vector<TS::Variable> current_vars =
            configs[i].get_vars();

        OpFileLoc << std::to_string(i) << ",";

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

void passnum_test() {
    size_t n = 100;

    for (size_t i = 0; i < n + 1; i++) {
        double ip_H2_frac = 1. / static_cast<double>(n) * static_cast<double>(i);

        double op_cg_loc = 1e10;
        double op_calc_mass = 1e10;
        double op_cg_loc_nofuel = 1e10;
        double op_calc_mass_nofuel = 1e10;
        double op_payload = 1e10;
        double op_M_JA1 = 1e10; // m
        double op_M_H2 = 1e10; // m
        int op_num_pass = 1e10;
        double op_tank_l = 1e10;
        bool op_vio_mass = false;
        bool op_vio_vol = false;

        // Calculate the hybrid BSFC
        double BSFC_hybrid_cruise = AircraftModel::calculate_hybrid_BSFC(ip_H2_frac, 6.1, 2050);

        double w_ratio = AircraftModel::breguet_prop_wratio(0.8, BSFC_hybrid_cruise, 15, 1400 * 1000);
        double w_fuel = 22000. * (1. - 1. / w_ratio);

        AircraftModel::compute_cg_loc_mass(480, w_fuel, ip_H2_frac, op_cg_loc, op_calc_mass, op_cg_loc_nofuel,
            op_calc_mass_nofuel, op_payload, op_M_JA1, op_M_H2, op_num_pass, op_tank_l, op_vio_mass, op_vio_vol);

        std::cout << ip_H2_frac << " - " << op_num_pass << "\n";
    }
}


int main()
{

    const bool TS = false;

    // Create a variable for the range
    std::array<double, 2> feas_reg_range = { 500, 2800 };
    std::vector<std::array<double, 2>> feas_regs_range = { feas_reg_range };
    const double start_range = 1400; // km
    const double stepsize_range = 400; // km
    const double stepsize_min_range = 100; // km
    const std::string name_range = "Range (km)";
    TS::Variable var_range(false, feas_regs_range, stepsize_min_range, start_range, stepsize_range,
        name_range);

    // Create a variable for the max. single engine power
    std::array<double, 2> feas_reg_Pmax = { 1500, 3500 }; //kW
    std::vector<std::array<double, 2>> feas_regs_Pmax = { feas_reg_Pmax };
    const double start_Pmax = 2050;
    const double stepsize_Pmax = 500;
    const double stepsize_min_Pmax = 50;
    const std::string name_Pmax = "Max. Single Engine Power (kW)";
    TS::Variable var_Pmax(false, feas_regs_Pmax, stepsize_min_Pmax, start_Pmax, stepsize_Pmax,
        name_Pmax);

    // Create a variable for the Cruise Altitude
    std::array<double, 2> feas_reg_h = { 4.500, 7.620 }; // km (15000 - 25000 ft)
    std::vector<std::array<double, 2>> feas_regs_h = { feas_reg_h };
    const double start_h = 6.100; // km (20000 ft)
    const double stepsize_h = 0.620; // km
    const double stepsize_min_h = 0.300; // km
    const std::string name_h = "Cruise Altitude (km)";
    TS::Variable var_h(false, feas_regs_h, stepsize_min_h, start_h, stepsize_h, name_h);

    // Create a variable for the Cruise Mach Number
    std::array<double, 2> feas_reg_M = { 0.4, 0.67 };
    std::vector<std::array<double, 2>> feas_regs_M = { feas_reg_M };
    const double start_M = 0.45;
    const double stepsize_M = 0.2;
    const double stepsize_min_M = 0.05;
    const std::string name_M = "Cruise Mach Number";
    TS::Variable var_M(false, feas_regs_M, stepsize_min_M, start_M, stepsize_M, name_M);

    if (TS) {

        // Initialize the vector containing the variables
        std::vector<TS::Variable> vars;

        // Create a variable for the Hydrogen power fraction
        std::array<double, 2> feas_reg_H2_Pfrac = { 0, 1 };
        std::vector<std::array<double, 2>> feas_regs_H2_Pfrac = { feas_reg_H2_Pfrac };
        const double start_H2_Pfrac = 0.5;
        const double stepsize_H2_Pfrac = 0.4;
        const double stepsize_min_H2_Pfrac = 0.05;
        const std::string name_H2_Pfrac = "Hydrogen Power Fraction";
        TS::Variable var_H2_Pfrac(false, feas_regs_H2_Pfrac, stepsize_min_H2_Pfrac, start_H2_Pfrac,
            stepsize_H2_Pfrac, name_H2_Pfrac);
        
        vars.push_back(var_range);
        vars.push_back(var_Pmax);
        vars.push_back(var_h);
        vars.push_back(var_M);
        vars.push_back(var_H2_Pfrac);

        // Make the initial point from the variables
        TS::Config initial_point(vars);

        // Specify the MDR Layers of Dominance
        MDR::DomRel first_layer(0, 1); // 3, 4
        MDR::DomRel second_layer(3, 6); // 0, 4
        MDR::DomRel third_layer(4, 4); // 1, 2
        std::vector<MDR::DomRel> dom_rels = { first_layer, second_layer }; //{ first_layer, second_layer, third_layer }

        // Specify the TS Parameters
        size_t STM_size = 7;
        double reduction_factor = 0.5;
        unsigned seed = 1;
        size_t INTENSIFY = 10;
        size_t DIVERSIFY = 15;
        size_t REDUCE = 25;
        size_t max_eval_num = 300;
        size_t HJ_num = 8;

        // Set up the simulation
        printf("Setting up Simulation\n");

        // Open Socket
        const char* IP = "192.168.0.93";//"192.168.1.150";     //IP Address of computer running X-Plane
        XPCSocket sock = openUDP(IP);
        float tVal[1];
        int tSize = 1;
        if (getDREF(sock, "sim/test/test_float", tVal, &tSize) < 0)
        {
            printf("Error establishing connection. Unable to read data from X-Plane.");
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
        Optimizer.print_visited_pts_loc_perf();
        Optimizer.print_pareto_front_layers();
    }
    else {

        const size_t n = 1000;
        std::vector<TS::Config> configs;

        for (size_t i = 0; i < n+1; i++) {

            // Initialize the vector containing the variables
            std::vector<TS::Variable> vars;

            // Create a variable for the Hydrogen power fraction
            std::array<double, 2> feas_reg_H2_Pfrac = { 0, 1 };
            std::vector<std::array<double, 2>> feas_regs_H2_Pfrac = { feas_reg_H2_Pfrac };
            const double start_H2_Pfrac = static_cast<double>(i) * 1. / static_cast<double>(n);
            const double stepsize_H2_Pfrac = 0.4;
            const double stepsize_min_H2_Pfrac = 0.05;
            const std::string name_H2_Pfrac = "Hydrogen Power Fraction";
            TS::Variable var_H2_Pfrac(false, feas_regs_H2_Pfrac, stepsize_min_H2_Pfrac, start_H2_Pfrac,
                stepsize_H2_Pfrac, name_H2_Pfrac);

            vars.push_back(var_range);
            vars.push_back(var_Pmax);
            vars.push_back(var_h);
            vars.push_back(var_M);
            vars.push_back(var_H2_Pfrac);

            // Make the initial point from the variables
            TS::Config initial_point(vars);

            AircraftEval::compute_f_nosim(initial_point, i);

            // Keep track of the points visited
            configs.push_back(initial_point);
        }

        std::string filename = "AllPoints";
        printconfigs(configs, filename);
    }
	
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
