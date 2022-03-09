
#include "../headers/ObjectiveFunction.h"

namespace AircraftEval {

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
        float simspeed_val = 3;
        sendDREF(sock, simspeed_dref, &simspeed_val, 1); // Send data

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

        // pauseSim
        pauseSim(sock, 1); // Sending 1 to pause	
        sleep(5); // Pause for 5 seconds

        // Unpause
        pauseSim(sock, 0); // Sending 0 to unpause
	}

    void reset_sim(XPCSocket sock, const double& ip_h, const double& ip_TAS) {
        
        // Convert inputs to float and also convert height to meters
        float height = 1000 * static_cast<float>(ip_h); // m
        float TAS = static_cast<float>(ip_TAS); // m/s
        
        // Set Location/Orientation (sendPOSI)
        // Set Up Position Array
        double POSI[7] = { 0.0 };

        POSI[0] = 51.875278627882849;
        POSI[1] = 0.22022808392539564;
        POSI[2] = 1.1 * height; // 10% higher to account for transient
        POSI[3] = -1.6440951824188232;
        POSI[4] = -0.14142291247844696;
        POSI[5] = 42.833587646484375;
        POSI[6] = 0.0;

        sendPOSI(sock, POSI, 7, 0);

        const char* reload_comm = "sim/operation/reload_aircraft_no_art";
        sendCOMM(sock, reload_comm);

        const char* speed_dref = "sim/flightmodel/position/local_vx"; // real DREF
        sendDREF(sock, speed_dref, &TAS, 1); // Send data

        const char* ap_state_dref = "sim/cockpit/autopilot/autopilot_state"; // AP State

        const char* ap_mode_dref = "sim/cockpit/autopilot/autopilot_mode"; // AP Mode
        float ap_mode_val = 2; // off=0, flight director=1, on=2
        sendDREF(sock, ap_mode_dref, &ap_mode_val, 1); // Send data

        // Convert the height to feet
        const float m_to_feet = 3.28084;
        height *= m_to_feet;
        const char* ap_alt_dref = "sim/cockpit/autopilot/altitude"; // AP Altitude
        sendDREF(sock, ap_alt_dref, &height, 1); // Send data

        const char* ap_vs_dref = "sim/cockpit/autopilot/vertical_velocity"; // AP Vertical Speed
        float ap_vs_val = 1000; // in ft/min above sea level
        sendDREF(sock, ap_vs_dref, &ap_vs_val, 1); // Send data

        const char* ap_hdg_dref = "sim/cockpit/autopilot/heading"; // AP Heading
        float ap_hdg_val = 43; // in ft/min above sea level
        sendDREF(sock, ap_hdg_dref, &ap_hdg_val, 1); // Send data

        // Simulate for 7 seconds
        sleep(14); 
    }

    void get_metrics(XPCSocket sock, double& op_L, double& op_D, double& op_Thrust, double& op_TAS,
        double& op_eta_prop) {
        
        int size = 1;

        float Lift = 1e10;
        const char* dref_lift = "sim/flightmodel/forces/fnrml_aero";
        getDREF(sock, dref_lift, &Lift, &size);
        op_L = static_cast<double>(Lift);

        float Drag = 1e10;
        const char* dref_drag = "sim/flightmodel/forces/faxil_aero";
        getDREF(sock, dref_drag, &Drag, &size);
        op_D = static_cast<double>(Drag);

        float Thrust = 1e10;
        const char* dref_thrust = "sim/flightmodel/forces/faxil_prop";
        getDREF(sock, dref_thrust, &Thrust, &size);
        op_Thrust = -static_cast<double>(Thrust);

        float TAS = 1e10;
        const char* dref_TAS = "sim/flightmodel/position/local_vx";
        getDREF(sock, dref_TAS, &TAS, &size);
        op_TAS = static_cast<double>(TAS);

        float eta_prop = 1e10;
        const char* dref_eta_prop = "sim/flightmodel/engine/POINT_prop_eff";
        getDREF(sock, dref_eta_prop, &eta_prop, &size);
        op_eta_prop = static_cast<double>(eta_prop);     
    }

    // Given an input configuration, evaluate its performance and
    // update the configuration with the performance metrics
    bool compute_f(TS::Config& ip_config) {

        //// THIS IS A PLACEHOLDER FUNCTION

        //std::vector<TS::Variable> variables = ip_config.get_vars();

        //double obj_A = 0;
        //double obj_B = 0;

        //for (size_t i = 0; i < variables.size(); i++) {
        //    double var_val = variables[i].get_val();
        //    obj_A += (var_val - 4) * var_val + 3;
        //    obj_B += 3 * sin(var_val * 10);//+= 5 - var_val;
        //}

        //// Formally Store Performance Metric A
        //MDR::MetricID idA("Metric A", 0);
        //MDR::PerfMetric perfA(idA, obj_A, true);

        //// Formally Store Performance Metric B
        //MDR::MetricID idB("Metric B", 1);
        //MDR::PerfMetric perfB(idB, obj_B / 3, true);

        //// Put them in a vector
        //std::vector<MDR::PerfMetric> perf_vect = { perfA, perfB };

        //// This is a bit silly but necessary
        //size_t zero = 0;
        //size_t one = 1;

        //// Make a Design object and assign it to the input configuration
        //MDR::Design performances(perf_vect, zero, zero, one);
        //ip_config.set_performances(performances);

        // Extract the optimization variables from the current configuration
        std::vector<TS::Variable> variables = ip_config.get_vars();
        const double ip_range = variables[0].get_val(); // km
        const double ip_H2_Pfrac = variables[1].get_val();
        const double ip_P_max = variables[2].get_val(); // kW
        const double ip_h = variables[3].get_val(); // km
        const double ip_M = variables[4].get_val();

        // Initialize the conversion constants
        const double kW_to_HP = 1.34102;

        // Initialize the aircraft address
        const std::string acf_filepath = 
            "D:\\Games\\X - Plane 11\\Aircraft\\Extra Aircraft\\ATR72 - 500\\ATR72.acf";

        // Initialize the true aircraft constants
        const double MTOW = 22800; // kg
        const double Raymer_L_D_max = 16.43;
        const double S_wing = 61; // m^2
        const double x_H2_tanks = 12.202; // m
        const double c_JA1 = 43.15; // MJ/kg
        const double c_H2 = 142; // MJ/kg
        const double emissions_per_kgJA1 = 3.16; // See https://www.offsetguide.org/understanding-carbon-offsets/air-travel-climate/climate-impacts-from-aviation/co2-emissions/

        // Initialize the variables that can be changed
        double eta_prop = 0.8;
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
        double op_groundrun = 1e10; // m
        double op_payfrac = 1e10;
        double op_NRG_paykm = 1e10; // MJ
        double op_emmiss_paykm = 1e10; // Tons CO2

        // Set-up UDP socket
        const char* IP = "128.232.250.212";     //IP Address of computer running X-Plane
        XPCSocket sock = openUDP(IP);
        float tVal[1];
        int tSize = 1;
        if (getDREF(sock, "sim/test/test_float", tVal, &tSize) < 0)
        {
            printf("Error establishing connecting. Unable to read data from X-Plane.");
            return false;
        }

        // Perform one "Tuning" iteration, then evaluate
        for (size_t i = 0; i < 2; i++) {
            // Calculate the fuel weight
            w_ratio = AircraftModel::breguet_prop_wratio(eta_prop, BSFC_hybrid_cruise, L_D, ip_range * 1000);
            w_fuel = mass_total * (1 - 1 / w_ratio);

            // Write the fuel data to the ACF file
            PlaneMakerTools::set_fuel_data(w_fuel, x_H2_tanks, ip_H2_Pfrac, acf_filepath);

            // Make the load computations
            bool mass_violation = false;
            bool volume_violation = false;
            AircraftModel::compute_cg_loc_mass(w_engine, w_fuel, ip_H2_Pfrac, x_cg, mass_total, x_cg_nofuel,
                mass_nofuel, mass_payload, mass_JA1, mass_violation, volume_violation);

            // Write the load data to the ACF file
            PlaneMakerTools::set_weight_data(x_cg_nofuel, mass_nofuel, acf_filepath);

            // Reset the simulator and let it run for 7 seconds
            reset_sim(sock, ip_h, TAS);

            get_metrics(sock, op_L, op_D, op_Thrust, op_TAS, eta_prop);

            // We will use the input L_D
            //if (i == 0) { L_D = op_L / op_D; }
            L_D = op_L / op_D;
        }
        
        // Evaluate and store the performance metrics        

        // Initialize the performance metric vector
        std::vector<MDR::PerfMetric> perf_vect;

        // Formally store the mission energy per payload km
        op_NRG_paykm = (mass_JA1 * c_JA1 + (w_fuel - mass_JA1) * c_H2) / (mass_payload * ip_range);
        MDR::MetricID NRG_id("NRG (MJ/cargokm)", 0);
        MDR::PerfMetric NRG_perf(NRG_id, op_NRG_paykm, true);
        perf_vect.push_back(NRG_perf);

        // Formally store the in-flight emissions per payload km
        op_emmiss_paykm = mass_JA1 * emissions_per_kgJA1 / (mass_payload * ip_range);
        MDR::MetricID emms_id("Emmissions (kgCO2/cargokm)", 1);
        MDR::PerfMetric emms_perf(emms_id, op_emmiss_paykm, true);
        perf_vect.push_back(emms_perf);

        // Formally store the payload fraction
        op_payfrac = mass_payload / mass_total;
        MDR::MetricID payfrac_id("Payload Fraction", 2);
        MDR::PerfMetric payfrac_perf(payfrac_id, op_payfrac, true);
        perf_vect.push_back(payfrac_perf);

        // Formally store the Lift Over Drag
        op_L_D = L_D;
        MDR::MetricID L_D_id("-L_D", 3);
        MDR::PerfMetric L_D_perf(L_D_id, -op_L_D, true);
        perf_vect.push_back(L_D_perf);

        // Formally store the ground run
        op_groundrun = AircraftModel::compute_ground_run_raymer(mass_total, S_wing, 1, 1, ip_P_max * kW_to_HP);
        MDR::MetricID grun_id("Ground Run (km)", 4);
        MDR::PerfMetric grun_perf(grun_id, op_groundrun, true);
        perf_vect.push_back(grun_perf);

        // Set the performances to the input configuration
        size_t zero = 0;
        size_t one = 1;
        MDR::Design performances(perf_vect, zero, zero, one);
        ip_config.set_performances(performances);

        return true;
    }

}
