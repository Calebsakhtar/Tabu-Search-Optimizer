
#include "../headers/ObjectiveFunction.h"

namespace AircraftEval {

    void display_aircraft_data(const double& x_CG_nofuel, const double& M_nofuel, 
        const double& ip_h, const double& ip_TAS, const double& ip_BSFC_full, 
        const double& ip_BSFC_full_low, const double& P_max, double M_fuel, 
        const double& H2_M_prop) {
    
    }

    // Compare two variables
    bool similar(const double& var1, const double& var2, const double& rel_tol) {

        const double rel_delta = 2. * abs(var1 - var2) / (abs(var1) + abs(var2));

        if (rel_delta < rel_tol) {
            return true;
        }
        else {
            return false;
        }
    }

	void init_simulator(XPCSocket sock) {
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

        // Set simulation speed
        const char* simspeed_dref = "sim/time/sim_speed"; // real DREF
        float simspeed_val = 3.;
        sendDREF(sock, simspeed_dref, &simspeed_val, 1); // Send data

        const char* speed_dref = "sim/flightmodel/position/local_vx"; // real DREF
        float speed_val = 143.377;
        sendDREF(sock, speed_dref, &speed_val, 1); // Send data

        const char* ap_off_dref = "sim/cockpit/autopilot/autopilot_mode"; // AP Mode
        float ap_off_val = 0.; // off=0, flight director=1, on=2
        sendDREF(sock, ap_off_dref, &ap_off_val, 1); // Send data

        const char* ap_state_dref = "sim/cockpit/autopilot/autopilot_state"; // AP State

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

        float ap_hdg2_val = 4; // wing level hold
        sendDREF(sock, ap_state_dref, &ap_hdg2_val, 1); // Send data
	}

    void reset_sim(XPCSocket sock, const double& ip_h, const double& ip_TAS) {
        // Resets the simulator (at the appropriate height ip_h and true airspeed ip_TAS).
        //
        // Height ip_h is in m
        // True Airspeed ip_TAS is in m/s
        
        // Convert inputs to float and also convert height to meters
        float height = 1000 * static_cast<float>(ip_h); // m
        float TAS = static_cast<float>(ip_TAS); // m/s
        
        // Set Location/Orientation (sendPOSI)
        // Set Up Position Array
        double POSI[7] = { 0.0 };

        POSI[0] = 51.875278627882849;
        POSI[1] = 0.22022808392539564;
        POSI[2] = height;
        POSI[3] = -1.6440951824188232;
        POSI[4] = -0.14142291247844696;
        POSI[5] = 42.833587646484375;
        POSI[6] = 0.0;

        sendPOSI(sock, POSI, 7, 0);

        // Set simulation speed
        const char* simspeed_dref = "sim/time/sim_speed"; // real DREF
        float simspeed_val = 3;
        sendDREF(sock, simspeed_dref, &simspeed_val, 1); // Send data

        const char* reload_comm = "sim/operation/reload_aircraft_no_art";
        sendCOMM(sock, reload_comm);

        const char* speed_dref = "sim/flightmodel/position/local_vx"; // real DREF
        sendDREF(sock, speed_dref, &TAS, 1); // Send data

        const char* ap_state_dref = "sim/cockpit/autopilot/autopilot_state"; // AP State

        const char* ap_mode_dref = "sim/cockpit/autopilot/autopilot_mode"; // AP Mode
        float ap_mode_val = 2; // off=0, flight director=1, on=2
        sendDREF(sock, ap_mode_dref, &ap_mode_val, 1); // Send data

        const char* ap_airspeed_dref = "sim/cockpit/autopilot/airspeed";
        double IAS = AircraftModel::TAS_to_IAS(ip_TAS, ip_h); // m/s
        float ap_airspeed_val = static_cast<float>(IAS) * 1.94384; // IAS, knots
        sendDREF(sock, ap_airspeed_dref, &ap_airspeed_val, 1); // Send data

        // Convert the height to feet
        const float m_to_feet = 3.28084;
        height *= m_to_feet;
        const char* ap_alt_dref = "sim/cockpit/autopilot/altitude"; // AP Altitude
        sendDREF(sock, ap_alt_dref, &height, 1); // Send data

        const char* ap_vs_dref = "sim/cockpit/autopilot/vertical_velocity"; // AP Vertical Speed
        float ap_vs_val = 1000; // in ft/min above sea level
        sendDREF(sock, ap_vs_dref, &ap_vs_val, 1); // Send data

        float ap_athr_val = 1; // change autothrottle
        sendDREF(sock, ap_state_dref, &ap_athr_val, 1); // Send data

        // Simulate for 7 seconds
        sleep(40); // 40
    }

    bool get_metrics(XPCSocket sock, double& op_L, double& op_D, double& op_Thrust, double& op_TAS,
        double& op_h, double& op_vz) {
        
        int size = 1;
        int comm_status = 0;

        float Lift = 1e10;
        const char* dref_lift = "sim/flightmodel/forces/lift_path_axis"; //fnrml_aero, N
        comm_status += getDREF(sock, dref_lift, &Lift, &size);
        op_L = static_cast<double>(Lift);

        float Drag = 1e10;
        const char* dref_drag = "sim/flightmodel/forces/drag_path_axis"; // faxil_aero, N
        comm_status += getDREF(sock, dref_drag, &Drag, &size);
        op_D = static_cast<double>(Drag);

        float Thrust = 1e10;
        const char* dref_thrust = "sim/flightmodel/forces/faxil_prop";
        comm_status += getDREF(sock, dref_thrust, &Thrust, &size);
        op_Thrust = static_cast<double>(Thrust);

        float TAS = 1e10; // m/s
        const char* dref_TAS = "sim/flightmodel/position/local_vx"; 
        comm_status += getDREF(sock, dref_TAS, &TAS, &size);
        op_TAS = static_cast<double>(TAS);

        float h = 1e10; // ft
        const char* dref_h = "sim/cockpit2/gauges/indicators/altitude_ft_pilot";
        comm_status += getDREF(sock, dref_h, &h, &size);
        op_h = static_cast<double>(h) / 3.281; // m

        float vz = 1e10; // m/s
        const char* dref_vz = "sim/flightmodel/position/local_vz";
        comm_status += getDREF(sock, dref_vz, &vz, &size);
        op_vz = static_cast<double>(vz);

        // If reading data from X-Plane has failed, return false
        if (comm_status < 0) { return false; }
        return true;

        //float eta_prop[8] = { 1e10, 1e10, 1e10, 1e10, 1e10, 1e10, 1e10, 1e10 };
        //const char* dref_eta_prop = "sim/flightmodel/engine/POINT_prop_eff";
        //getDREF(sock, dref_eta_prop, eta_prop, &size);
        //float eta_prop2 = eta_prop[0];
        //op_eta_prop = static_cast<double>(eta_prop2);     
    }

    void write_current_aircraft_data(const TS::Config& config, const int& p) {

        // Make the file termination
        std::string filename = std::to_string(p);

        // Bad way to pad with zeroes
        if (p < 10) {
            filename = "0000" + filename;
        }
        else if (p < 100) {
            filename = "000" + filename;
        }
        else if (p < 1000) {
            filename = "00" + filename;
        }
        else if (p < 10000) {
            filename = "0" + filename;
        }

        std::string filename_perf = "Results/Aircraft" + filename + "_Perf.csv";
        std::string filename_coords = "Results/Aircraft" + filename + "_Coords.csv";




        // Create and open a text file to store the coordinates of all the Visited Points
        std::ofstream OpFileCoords(filename_coords);

        size_t vars_size = config.get_vars().size();

        for (size_t i = 0; i < vars_size - 1; i++) {
            OpFileCoords << "Variable" << std::to_string(i) << ",";
        }

        OpFileCoords << "Variable" << std::to_string(vars_size -1);


        OpFileCoords << "\n";

        std::vector<TS::Variable> current_coords = config.get_vars();

        for (size_t j = 0; j < current_coords.size() - 1; j++) {
            OpFileCoords << std::to_string(current_coords[j].get_val()) << ",";
        }

        OpFileCoords << std::to_string(current_coords[current_coords.size() - 1].get_val());

        // Close the file
        OpFileCoords.close();




        // Create and open a text file to store the performances of all the Visited Points
        std::ofstream OpFilePerf(filename_perf);

        size_t perf_size = config.get_performances().get_perf_vector().size();

        for (size_t i = 0; i < perf_size; i++) {
            OpFilePerf << "Objective" << std::to_string(i) << ",";
        }

        OpFilePerf << "\n";

        std::vector<MDR::PerfMetric> current_perfs = config.get_performances().get_perf_vector();

        for (size_t j = 0; j < current_perfs.size() - 1; j++) {
            OpFilePerf << std::to_string(current_perfs[j].get_metric_val()) << ",";
        }

        OpFilePerf << std::to_string(current_perfs[current_perfs.size() - 1].get_metric_val());

        // Close the file
        OpFilePerf.close();
    };


    // Given an input configuration, evaluate its performance and
    // update the configuration with the performance metrics
    bool compute_f(TS::Config& ip_config, const XPCSocket sock, const size_t num_f_evals) {

        std::cout << "\n" << "Simulating Aircraft " << std::to_string(num_f_evals) << "\n";

        // Extract the optimization variables from the current configuration
        std::vector<TS::Variable> variables = ip_config.get_vars();
        const double ip_range = variables[0].get_val(); // 1400 km
        const double ip_P_max = variables[1].get_val(); // 2050 kW
        const double ip_h = variables[2].get_val(); // 6.096 km
        const double ip_M = variables[3].get_val(); // 0.456
        const double ip_H2_Pfrac = variables[4].get_val();

        // Initialize the conversion constants
        const double kW_to_HP = 1.34102;

        // Initialize the aircraft address
        std::string acf_filepath = 
            "C:\\CalebData\\Games\\X-Plane 11\\Aircraft\\Extra Aircraft\\ATR72-500\\ATR72.acf";

        // Initialize the true aircraft constants
        const double MTOW = 22000; // kg
        const double Raymer_L_D_max = 16.43;
        const double S_wing = 61; // m^2
        const double x_H2_tanks = 12.202; // m
        const double c_JA1 = 43.0; // MJ/kg (specific energy = LCV)
        const double c_H2 = 121.1; // MJ/kg (specific energy)
        const double emissions_per_kgH2 = 0.; // 0 for green, 0.97 for blue, 9.71 for grey
        const double emissions_per_kgJA1 = 3.16; // See https://www.icao.int/environmental-protection/CarbonOffset/Documents/Methodology%20ICAO%20Carbon%20Calculator_v10-2017.pdf
        const double eta_prop = 0.8;

        // Initialize the variables that can be changed
        double L_D = 0.8 * Raymer_L_D_max;
        double mass_total = MTOW;
        double BSFC_hybrid_cruise = 0;
        double w_ratio = 0;
        double w_fuel = 0; // kg
        double w_engine = 0; // kg
        double TAS = 0; // m/s
        double x_cg = 0; // m
        double x_cg_nofuel = 0; // m
        double mass_nofuel = 0; // kg
        double mass_payload = 0; // kg
        double mass_JA1 = 0; // kg
        double mass_H2_net = 0; //kg
        double H2_mprop = 0;
        bool mass_violation = false;
        bool volume_violation = false;
        bool read_violation = false;
        bool ss_violation = false;

        // Calculate the ISA values
        double ISA_T = 0;
        double ISA_a = 0;
        double ISA_P = 0;
        double ISA_rho = 0;
        double ISA_visc = 0;
        AircraftModel::ISA(ip_h, ISA_T, ISA_a, ISA_P, ISA_rho, ISA_visc);

        // Calculate the TAS of the aircraft
        TAS = ISA_a * ip_M; // m/s

        // Calculate the hybrid BSFC
        BSFC_hybrid_cruise = AircraftModel::calculate_hybrid_BSFC(ip_H2_Pfrac, ip_h, ip_P_max);

        // Compute the engine weight
        w_engine = AircraftModel::correl_turboprop_mass(ip_P_max);

        // Write the engine data to the ACF file
        // Note: the BSFC at low altitude is equal to that at high altitude since it is not currently needed
        PlaneMakerTools::set_engine_data(BSFC_hybrid_cruise, BSFC_hybrid_cruise, ip_P_max, acf_filepath);

        // Initialise the outputs
        double op_L = 1e10;
        double op_D = 1e10;
        double op_Thrust = 1e10;
        double op_TAS = 1e10;
        double op_L_D = 1e10;
        double op_payfrac = 1e10;
        double op_NRG = 1e10; // MJ
        double op_NRG_paykm = 1e10; 
        double op_NRG_paxkm = 1e10;
        double op_emmiss = 1e10; // Tons CO2
        double op_emmiss_paykm = 1e10;
        double op_emmiss_paxkm = 1e10;
        int op_num_pass = 1;
        double op_tank_l = 1e10;
        double op_h = 0;
        double op_vz = 1e10;

        // Perform one "Tuning" iteration, then evaluate
        for (size_t i = 0; i < 2; i++) {
            // Calculate the fuel weight
            w_ratio = AircraftModel::breguet_prop_wratio(eta_prop, BSFC_hybrid_cruise, L_D, ip_range * 1000);
            w_fuel = mass_total * (1. - 1. / w_ratio);


            // Make the load computations
            AircraftModel::compute_cg_loc_mass(w_engine, w_fuel, ip_H2_Pfrac, x_cg, mass_total, x_cg_nofuel,
                mass_nofuel, mass_payload, mass_JA1, mass_H2_net, op_num_pass, op_tank_l, mass_violation, 
                volume_violation);

            // Check for an appropriate kerosene mass
            if (mass_JA1 > 5000) {
                mass_violation = true;
            }

            // Check for mass and volume violations
            if (mass_violation || volume_violation) {

                if (mass_violation || mass_JA1 > 5000) {
                    std::cout << "Mass Violation in Aircraft " << std::to_string(num_f_evals) << "\n";
                }

                if (volume_violation) {
                    std::cout << "Volume Violation in Aircraft " << std::to_string(num_f_evals) << "\n";
                }

                break;
            }

            // Check for CG violations

            // Write the load data to the ACF file
            PlaneMakerTools::set_weight_data(x_cg_nofuel, mass_nofuel, acf_filepath);

            // Write the fuel data to the ACF file
            H2_mprop = (w_fuel - mass_JA1) / w_fuel;
            PlaneMakerTools::set_fuel_data(w_fuel, x_H2_tanks, H2_mprop, acf_filepath);

            // Reset the simulator and let it run for a while to achieve steady-state
            reset_sim(sock, ip_h, TAS);

            read_violation = not get_metrics(sock, op_L, op_D, op_Thrust, op_TAS, op_h, op_vz);

            if (read_violation) {
                // Try again
                read_violation = not get_metrics(sock, op_L, op_D, op_Thrust, op_TAS, op_h, op_vz);

                if (read_violation) {
                    std::cout << "Read Violation in Aircraft " << std::to_string(num_f_evals) << "\n";
                    break;
                }
            }

            // Check to see whether the aircraft is in steady state
            ss_violation = !similar(abs(op_TAS),abs(TAS), 0.05);
            ss_violation &= (op_vz > 0.26);
            ss_violation &= !similar(abs(ip_h), abs(op_h), 0.05);

            if (ss_violation) {
                std::cout << "Steady-State Violation in Aircraft " << std::to_string(num_f_evals) << "\n";
                break;
            }

            L_D = op_L / op_D;
        }

        // Evaluate and store the performance metrics        

        // Initialize the performance metric vector
        std::vector<MDR::PerfMetric> perf_vect;

        if (mass_violation || volume_violation || read_violation || ss_violation) {
            op_L = 1e10;
            op_D = 1e10;
            op_Thrust = 1e10;
            op_TAS = 1e10;
            op_L_D = -1e10;
            op_payfrac = -1e10;
            mass_total = 1e10;
            op_NRG = 1e10; // MJ
            op_NRG_paykm = 1e10;
            op_NRG_paxkm = 1e10;
            op_emmiss = 1e10; // Tons CO2
            op_emmiss_paykm = 1e10;
            op_emmiss_paxkm = 1e10;
            op_num_pass = 1;
        }
        else {
            if (mass_payload <= 1e-6) {
                op_NRG = 1e6;
                op_NRG_paykm = 1e6;
            }
            else {
                op_NRG = mass_JA1 * c_JA1 + (w_fuel - mass_JA1) * c_H2;

                if (op_NRG < 0 || op_L_D < 0 || op_payfrac < 0 || mass_total < 0) {
                    op_L = 1e10;
                    op_D = 1e10;
                    op_Thrust = 1e10;
                    op_TAS = 1e10;
                    op_L_D = -1e10;
                    op_payfrac = -1e10;
                    mass_total = 1e10;
                    op_NRG = 1e10; // MJ
                    op_NRG_paykm = 1e10;
                    op_NRG_paxkm = 1e10;
                    op_emmiss = 1e10; // Tons CO2
                    op_emmiss_paykm = 1e10;
                    op_emmiss_paxkm = 1e10;
                    op_num_pass = 1;
                }
            }
            
            op_payfrac = mass_payload / mass_total;

            op_emmiss = mass_JA1 * emissions_per_kgJA1 + mass_H2_net * emissions_per_kgH2;
            op_emmiss_paykm = op_emmiss / (mass_payload * ip_range);
            op_emmiss_paxkm = op_emmiss / (op_num_pass * ip_range);

            op_NRG_paykm = op_NRG / (mass_payload * ip_range);
            op_NRG_paxkm = op_NRG / (op_num_pass * ip_range);

            op_L_D = L_D;
        }
        
        // Formally store the mission energy
        MDR::MetricID NRG_id("NRG (MJ)", 0);
        MDR::PerfMetric NRG_perf(NRG_id, op_NRG, true);
        perf_vect.push_back(NRG_perf);

        // Formally store the mission energy per payload km
        MDR::MetricID NRG_cargo_id("NRG (kJ/cargokm)", 1);
        MDR::PerfMetric NRG_cargo_perf(NRG_cargo_id, 1e3 * op_NRG_paykm, true);
        perf_vect.push_back(NRG_cargo_perf);

        // Formally store the mission energy per passenger km
        MDR::MetricID NRG_pax_id("NRG (MJ/paxkm)", 2);
        MDR::PerfMetric NRG_pax_perf(NRG_pax_id, op_NRG_paxkm, true);
        perf_vect.push_back(NRG_pax_perf);

        // Formally store the payload fraction
        MDR::MetricID payfrac_id("-Payload Fraction", 3);
        MDR::PerfMetric payfrac_perf(payfrac_id, -op_payfrac, true);
        perf_vect.push_back(payfrac_perf);

        // Formally store the total mass
        MDR::MetricID totalm_id("Total Mass (kg)", 4);
        MDR::PerfMetric totalm_perf(totalm_id, mass_total, true);
        perf_vect.push_back(totalm_perf);

        // Formally store the in-flight emissions per payload km
        MDR::MetricID emms_id("Emmissions (kg CO2)", 5);
        MDR::PerfMetric emms_perf(emms_id, op_emmiss, true);
        perf_vect.push_back(emms_perf);
        
        // Formally store the in-flight emissions per payload km
        MDR::MetricID emms_cargo_id("Emmissions (gCO2/cargokm)", 6);
        MDR::PerfMetric emms_cargo_perf(emms_cargo_id, op_emmiss_paykm*1000., true);
        perf_vect.push_back(emms_cargo_perf);
        
        // Formally store the in-flight emissions per payload km
        MDR::MetricID emms_pax_id("Emmissions (kgCO2/paxkm)", 7);
        MDR::PerfMetric emms_pax_perf(emms_pax_id, op_emmiss_paxkm, true);
        perf_vect.push_back(emms_pax_perf);

        // Formally store the Lift Over Drag
        MDR::MetricID L_D_id("-L_D", 8);
        MDR::PerfMetric L_D_perf(L_D_id, -op_L_D, true);
        perf_vect.push_back(L_D_perf);

        // Formally store the Number of Passengers
        MDR::MetricID n_pass_id("-Number of Passengers", 9);
        MDR::PerfMetric n_pass_perf(n_pass_id, -op_num_pass, true);
        perf_vect.push_back(n_pass_perf);

        // Formally store the Length of the H2 tank
        MDR::MetricID l_H2_id("Length of Hydrogen Tank", 10);
        MDR::PerfMetric l_H2_perf(l_H2_id, op_tank_l, true);
        perf_vect.push_back(l_H2_perf);

        // Formally store the Total Mass of Hydrogen Carried
        MDR::MetricID M_H2_id("Mass of Hydrogen Carried (kg)", 11);
        MDR::PerfMetric M_H2_perf(M_H2_id, mass_H2_net, true);
        perf_vect.push_back(M_H2_perf);

        // Set the performances to the input configuration
        size_t zero = 0;
        size_t one = 1;
        MDR::Design performances(perf_vect, zero, zero, one);
        ip_config.set_performances(performances);

        write_current_aircraft_data(ip_config, num_f_evals);

        if (mass_violation || volume_violation || ss_violation) {
            return false;
        }

        return true;
    }

    // Given an input configuration, evaluate its performance and
    // update the configuration with the performance metrics
    // No X-Plane simulation is used
    bool compute_f_nosim(TS::Config& ip_config, const size_t num_f_evals) {

        std::cout << "\n" << "Simulating Aircraft " << std::to_string(num_f_evals) << "\n";

        // Extract the optimization variables from the current configuration
        std::vector<TS::Variable> variables = ip_config.get_vars();
        const double ip_range = variables[0].get_val(); // 1400 km
        const double ip_P_max = variables[1].get_val(); // 2050 kW
        const double ip_h = variables[2].get_val(); // 6.096 km
        const double ip_M = variables[3].get_val(); // 0.456
        const double ip_H2_Pfrac = variables[4].get_val();

        // Initialize the conversion constants
        const double kW_to_HP = 1.34102;

        // Initialize the true aircraft constants
        const double MTOW = 22000; // kg
        const double Raymer_L_D_max = 16.43;
        const double S_wing = 61; // m^2
        const double x_H2_tanks = 12.202; // m
        const double c_JA1 = 43.0; // MJ/kg (specific energy = LCV)
        const double c_H2 = 121.1; // MJ/kg (specific energy)
        const double emissions_per_kgH2 = 0.; // 0 for green, 0.97 for blue, 9.71 for grey
        const double emissions_per_kgJA1 = 3.16;// See https://www.icao.int/environmental-protection/CarbonOffset/Documents/Methodology%20ICAO%20Carbon%20Calculator_v10-2017.pdf
        const double eta_prop = 0.8;

        // Initialize the variables that can be changed
        double L_D = 15; //0.8 * Raymer_L_D_max;
        double mass_total = MTOW;
        double BSFC_hybrid_cruise = 0;
        double w_ratio = 0;
        double w_fuel = 0; // kg
        double w_engine = 0; // kg
        double TAS = 0; // m/s
        double x_cg = 0; // m
        double x_cg_nofuel = 0; // m
        double mass_nofuel = 0; // kg
        double mass_payload = 0; // kg
        double mass_JA1 = 0; // kg
        double mass_H2_net = 0; //kg
        double H2_mprop = 0;
        bool mass_violation = false;
        bool volume_violation = false;

        // Calculate the ISA values
        double ISA_T = 0;
        double ISA_a = 0;
        double ISA_P = 0;
        double ISA_rho = 0;
        double ISA_visc = 0;
        AircraftModel::ISA(ip_h, ISA_T, ISA_a, ISA_P, ISA_rho, ISA_visc);

        // Calculate the TAS of the aircraft
        TAS = ISA_a * ip_M; // m/s

        // Calculate the hybrid BSFC
        BSFC_hybrid_cruise = AircraftModel::calculate_hybrid_BSFC(ip_H2_Pfrac, ip_h, ip_P_max);

        // Compute the engine weight
        w_engine = AircraftModel::correl_turboprop_mass(ip_P_max);

        // Initialise the outputs
        double op_L = 1e10;
        double op_D = 1e10;
        double op_Thrust = 1e10;
        double op_TAS = 1e10;
        double op_L_D = 1e10;
        double op_payfrac = 1e10;
        double op_NRG = 1e10; // MJ
        double op_NRG_paykm = 1e10;
        double op_NRG_paxkm = 1e10;
        double op_emmiss = 1e10; // Tons CO2
        double op_emmiss_paykm = 1e10;
        double op_emmiss_paxkm = 1e10;
        int op_num_pass = 1;
        double op_tank_l = 1e10;
        double op_h = 0.;
        double op_vz = 1e10;

        // Perform one "Tuning" iteration, then evaluate
        for (size_t i = 0; i < 2; i++) {
            // Calculate the fuel weight
            w_ratio = AircraftModel::breguet_prop_wratio(eta_prop, BSFC_hybrid_cruise, L_D, ip_range * 1000);
            w_fuel = mass_total * (1. - 1. / w_ratio);


            // Make the load computations
            AircraftModel::compute_cg_loc_mass(w_engine, w_fuel, ip_H2_Pfrac, x_cg, mass_total, x_cg_nofuel,
                mass_nofuel, mass_payload, mass_JA1, mass_H2_net, op_num_pass, op_tank_l, mass_violation, 
                volume_violation);

            // Check for an appropriate kerosene mass
            if (mass_JA1 > 5000) {
                mass_violation = true;
            }

            // Check for mass and volume violations
            if (mass_violation || volume_violation) {

                if (mass_violation || mass_JA1 > 5000) {
                    std::cout << "Mass Violation in Aircraft " << std::to_string(num_f_evals) << "\n";
                }

                if (volume_violation) {
                    std::cout << "Volume Violation in Aircraft " << std::to_string(num_f_evals) << "\n";
                }

                break;
            }
        }

        // Evaluate and store the performance metrics        

        // Initialize the performance metric vector
        std::vector<MDR::PerfMetric> perf_vect;

        if (mass_violation || volume_violation) {
            op_L = 1e10;
            op_D = 1e10;
            op_Thrust = 1e10;
            op_TAS = 1e10;
            op_L_D = -1e10;
            op_payfrac = -1e10;
            mass_total = 1e10;
            op_NRG = 1e10; // MJ
            op_NRG_paykm = 1e10;
            op_NRG_paxkm = 1e10;
            op_emmiss = 1e10; // Tons CO2
            op_emmiss_paykm = 1e10;
            op_emmiss_paxkm = 1e10;
            op_num_pass = 1;
        }
        else {
            if (mass_payload <= 1e-6) {
                op_NRG = 1e6;
                op_NRG_paykm = 1e6;
            }
            else {
                op_NRG = mass_JA1 * c_JA1 + (w_fuel - mass_JA1) * c_H2;

                if (op_NRG < 0 || op_L_D < 0 || op_payfrac < 0 || mass_total < 0) {
                    op_L = 1e10;
                    op_D = 1e10;
                    op_Thrust = 1e10;
                    op_TAS = 1e10;
                    op_L_D = -1e10;
                    op_payfrac = -1e10;
                    mass_total = 1e10;
                    op_NRG = 1e10; // MJ
                    op_NRG_paykm = 1e10;
                    op_NRG_paxkm = 1e10;
                    op_emmiss = 1e10; // Tons CO2
                    op_emmiss_paykm = 1e10;
                    op_emmiss_paxkm = 1e10;
                    op_num_pass = 1;
                }
            }

            op_payfrac = mass_payload / mass_total;

            op_emmiss = mass_JA1 * emissions_per_kgJA1 + mass_H2_net * emissions_per_kgH2;
            op_emmiss_paykm = op_emmiss / (mass_payload * ip_range);
            op_emmiss_paxkm = op_emmiss / (op_num_pass * ip_range);

            op_NRG_paykm = op_NRG / (mass_payload * ip_range);
            op_NRG_paxkm = op_NRG / (op_num_pass * ip_range);

            op_L_D = L_D;
        }

        // Formally store the mission energy
        MDR::MetricID NRG_id("NRG (MJ)", 0);
        MDR::PerfMetric NRG_perf(NRG_id, op_NRG, true);
        perf_vect.push_back(NRG_perf);

        // Formally store the mission energy per payload km
        MDR::MetricID NRG_cargo_id("NRG (kJ/cargokm)", 1);
        MDR::PerfMetric NRG_cargo_perf(NRG_cargo_id, 1e3 * op_NRG_paykm, true);
        perf_vect.push_back(NRG_cargo_perf);

        // Formally store the mission energy per passenger km
        MDR::MetricID NRG_pax_id("NRG (MJ/paxkm)", 2);
        MDR::PerfMetric NRG_pax_perf(NRG_pax_id, op_NRG_paxkm, true);
        perf_vect.push_back(NRG_pax_perf);

        // Formally store the payload fraction
        MDR::MetricID payfrac_id("-Payload Fraction", 3);
        MDR::PerfMetric payfrac_perf(payfrac_id, -op_payfrac, true);
        perf_vect.push_back(payfrac_perf);

        // Formally store the total mass
        MDR::MetricID totalm_id("Total Mass (kg)", 4);
        MDR::PerfMetric totalm_perf(totalm_id, mass_total, true);
        perf_vect.push_back(totalm_perf);

        // Formally store the in-flight emissions per payload km
        MDR::MetricID emms_id("Emmissions (kg CO2)", 5);
        MDR::PerfMetric emms_perf(emms_id, op_emmiss, true);
        perf_vect.push_back(emms_perf);

        // Formally store the in-flight emissions per payload km
        MDR::MetricID emms_cargo_id("Emmissions (gCO2/cargokm)", 6);
        MDR::PerfMetric emms_cargo_perf(emms_cargo_id, op_emmiss_paykm * 1000., true);
        perf_vect.push_back(emms_cargo_perf);

        // Formally store the in-flight emissions per payload km
        MDR::MetricID emms_pax_id("Emmissions (kgCO2/paxkm)", 7);
        MDR::PerfMetric emms_pax_perf(emms_pax_id, op_emmiss_paxkm, true);
        perf_vect.push_back(emms_pax_perf);

        // Formally store the Lift Over Drag
        MDR::MetricID L_D_id("-L_D", 8);
        MDR::PerfMetric L_D_perf(L_D_id, -op_L_D, true);
        perf_vect.push_back(L_D_perf);

        // Formally store the Number of Passengers
        MDR::MetricID n_pass_id("-Number of Passengers", 9);
        MDR::PerfMetric n_pass_perf(n_pass_id, -op_num_pass, true);
        perf_vect.push_back(n_pass_perf);

        // Formally store the Length of the H2 tank
        MDR::MetricID l_H2_id("Length of Hydrogen Tank", 10);
        MDR::PerfMetric l_H2_perf(l_H2_id, op_tank_l, true);
        perf_vect.push_back(l_H2_perf);

        // Formally store the Total Mass of Hydrogen Carried
        MDR::MetricID M_H2_id("Mass of Hydrogen Carried (kg)", 11);
        MDR::PerfMetric M_H2_perf(M_H2_id, mass_H2_net, true);
        perf_vect.push_back(M_H2_perf);

        // Set the performances to the input configuration
        size_t zero = 0;
        size_t one = 1;
        MDR::Design performances(perf_vect, zero, zero, one);
        ip_config.set_performances(performances);

        write_current_aircraft_data(ip_config, num_f_evals);

        if (mass_violation || volume_violation) {
            return false;
        }

        return true;
    }

}
