// TabuSearch.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <array>
#include <fstream>

#include "headers/Variable.h"
#include "headers/Config.h"
#include "headers/STM.h"
#include "headers/MDROptimizer.h"
#include "../MDR-Test/MDR Test Project/headers/ReadDesigns.h"

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

    Optimizer.perform_optimization();

    auto result_MTM = Optimizer.retreive_MTM();
    auto all_pts = Optimizer.retrieve_all_pts();

    // Create and open a text file to store the Coordinates of all the Visited Points
    std::ofstream MyFile3("PointCoords.csv");

    if (all_pts.size() > 0) {

        auto vars_size  =
            all_pts[0].get_vars().size();

        MyFile3 << "PointID";

        for (size_t i = 0; i < vars_size; i++) {
            MyFile3 << ",Variable" << std::to_string(i + 1);
        }

        MyFile3 << "\n";

        for (size_t i = 0; i < all_pts.size(); i++) {
            std::vector<TS::Variable> current_vars =
                all_pts[i].get_vars();

            MyFile3 << std::to_string(i + 1) << ",";

            for (size_t j = 0; j < current_vars.size() - 1; j++) {
                MyFile3 << std::to_string(current_vars[j].get_val()) << ",";
            }

            MyFile3 << std::to_string(current_vars[current_vars.size() - 1].get_val()) << "\n";
        }
    }

    // Close the file
    MyFile3.close();

    // Create and open a text file to store the ParetoFront in
    std::ofstream MyFile2("AllPoints.csv");

    if (all_pts.size() > 0) {

        size_t perf_size2 =
            all_pts[0].get_performances().get_perf_vector().size();

        MyFile2 << "PointID";

        for (size_t i = 0; i < perf_size2; i++) {
            MyFile2 << ",Objective" << std::to_string(i + 1);
        }

        MyFile2 << "\n";
        
        for (size_t i = 0; i < all_pts.size(); i++) {
            std::vector<MDR::PerfMetric> current_perfs2 = 
                all_pts[i].get_performances().get_perf_vector();
            
            MyFile2 << std::to_string(i+1) << ",";

            for (size_t j = 0; j < current_perfs2.size()-1; j++) {
                MyFile2 << std::to_string(current_perfs2[j].get_metric_val()) << ",";
            }

            MyFile2 << std::to_string(current_perfs2[current_perfs2.size() - 1].get_metric_val()) << "\n";
        }
    }

    // Close the file
    MyFile2.close();


    // Create and open a text file to store the ParetoFront in
    std::ofstream MyFile("Points.csv");

    if (result_MTM.size() > 0) {

        size_t perf_size =
            result_MTM[0].get_performances().get_perf_vector().size();

        MyFile << "MTMID";

        for (size_t i = 0; i < perf_size; i++) {
            MyFile << ",Objective" << std::to_string(i + 1);
        }

        MyFile << "\n";

        for (size_t i = 0; i < result_MTM.size(); i++) {
            std::vector<MDR::PerfMetric> current_perfs =
                result_MTM[i].get_performances().get_perf_vector();

            MyFile << std::to_string(i + 1) << ",";

            for (size_t j = 0; j < current_perfs.size() - 1; j++) {
                MyFile << std::to_string(current_perfs[j].get_metric_val()) << ",";
            }

            MyFile << std::to_string(current_perfs[current_perfs.size() - 1].get_metric_val()) << "\n";
        }
    }

    // Close the file
    MyFile.close();

    // Finish the test
    std::cout << "Hello World!\n";

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
