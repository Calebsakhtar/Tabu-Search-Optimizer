// TabuSearch.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include "headers/Variable.h"

int main()
{
    // Create two variables
    std::vector<double> feasible_region = { 0, 1 };
    std::vector<std::vector<double>> feasible_regions = { feasible_region };
    TS::Variable var1(false, feasible_regions, 1, 10, 1, "a");
    TS::Variable var2(false, feasible_regions, 1, 9, 1, "b");

    // Test whether the "<" operator works as expected
    if (!(var1 < var2)) {
        std::cout << "This works!\n\n";
    }
    
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
