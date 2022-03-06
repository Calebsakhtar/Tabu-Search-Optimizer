#include <vector>
#include <string>
#include <math.h>

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

    void reset_sim(XPCSocket sock) {
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

        // pauseSim
        pauseSim(sock, 1); // Sending 1 to pause	
        sleep(5); // Pause for 5 seconds

        // Unpause
        pauseSim(sock, 0); // Sending 0 to unpause
    }

    // Given an input configuration, evaluate its performance and
    // update the configuration with the performance metrics
    void compute_f(TS::Config& ip_config) {

        // THIS IS A PLACEHOLDER FUNCTION

        std::vector<TS::Variable> variables = ip_config.get_vars();

        double obj_A = 0;
        double obj_B = 0;

        for (size_t i = 0; i < variables.size(); i++) {
            double var_val = variables[i].get_val();
            obj_A += (var_val - 4) * var_val + 3;
            obj_B += 3 * sin(var_val * 10);//+= 5 - var_val;
        }

        // Formally Store Performance Metric A
        MDR::MetricID idA("Metric A", 0);
        MDR::PerfMetric perfA(idA, obj_A, true);

        // Formally Store Performance Metric B
        MDR::MetricID idB("Metric B", 1);
        MDR::PerfMetric perfB(idB, obj_B / 3, true);

        // Put them in a vector
        std::vector<MDR::PerfMetric> perf_vect = { perfA, perfB };

        // This is a bit silly but necessary
        size_t zero = 0;
        size_t one = 1;

        // Make a Design object and assign it to the input configuration
        MDR::Design performances(perf_vect, zero, zero, one);
        ip_config.set_performances(performances);
    }

}
