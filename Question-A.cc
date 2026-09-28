// Part A: This is an extension task that requires you to decode sensor data from CAN log files.
// CAN (Controller Area Network) is a communication standard used in automotive applications (including Redback cars)
// to allow communication between sensors and controllers.
//
// Your Task: Using the signal definitions in SteeringBench.dbc, read each CAN capture in data/
// and turn it into a CSV with one row per decoded frame:
// t,u_commanded,y_measured
// eg:
// 0,15.0,0.0
// 0.005,15.0,0.0
// ...
// where t is the frame timestamp minus the first kept frame's timestamp (s), u_commanded is
// the decoded CmdAngularRate (deg/s), and y_measured is the decoded MeasuredAngle (deg).
// The above values are not real numbers; they are only there to show the expected data output format.
// Do this for all three captures:
// data/step_test.log       ->  data/step_test.csv
// data/reversal_test.log   ->  data/reversal_test.csv
// data/deadband_test.log   ->  data/deadband_test.csv
//
// The Row type, writeCsv(), and main() below are provided -- they loop the three logs, call your
// decodeLog(), and write the CSV in exactly the format above. You just need to implement decodeLog().
//
// You do not need to use any external libraries. Use the resources below to understand how to
// extract sensor data.
// Hint: Think about manual bit masking and shifting, data types required,
// what formats are used to represent values, etc.
// Resources:
// https://www.csselectronics.com/pages/can-bus-simple-intro-tutorial
// https://www.csselectronics.com/pages/can-dbc-file-database-intro
//
// Sanity check: plot your CSVs (python3 plot_data.py) and compare against the pre-plotted
// data/*.png files -- they should match.
//
// Build & run (from the TA/ folder):
//     c++ -std=c++17 Question-A.cc -o decode
//     ./decode

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    // constants
    double u_scale = 0.1;
    double u_offset = 0.0;
    
    double y_scale = 0.1;
    double y_offset = 0.0;


    std::vector<Row> rows;

    std::ifstream log_file(path);
    std::string timestamp;
    std::string interface;
    std::string log;
    double start_time = -1.0;
    while (log_file >> timestamp >> interface >> log)
    {
        // the reason im not just using 0-2 is because i searched up how long a can id is and it can be variable
        // as such to ensure compatibility with various functions i just used hash to mark the end
        auto hash_loc = log.find('#');
        auto can_id = log.substr(0, hash_loc);

        // if the can id is irrelevant
        if (can_id != "200")
        {
            continue;
        }

        // gets the time stamp
        auto curr_time = std::stod(timestamp.substr(1));
        if (start_time < 0) {
            start_time = curr_time;
        }
        
        //gets the data in a usable format
        auto hex_data = log.substr(hash_loc + 1);

        // get the bytes in a integer format
        uint8_t b0 = std::stoul(hex_data.substr(0, 2), nullptr, 16); // the 16 confirms that a hex number is being converted
        uint8_t b1 = std::stoul(hex_data.substr(2, 2), nullptr, 16);
        uint8_t b2 = std::stoul(hex_data.substr(4, 2), nullptr, 16);
        uint8_t b3 = std::stoul(hex_data.substr(6, 2), nullptr, 16);

        // combine the bytes (little endian) to form the data
        int16_t unscaled_y = static_cast<int16_t>((b1 << 8) | b0);
        int16_t unscaled_u = static_cast<int16_t>((b3 << 8) | b2);

        // apply scale and offset (i made these constants for easy changability)
        double y_measured  = (unscaled_y * y_scale) + y_offset;
        double u_commanded = (unscaled_u * u_scale) + u_offset;

        rows.push_back({curr_time - start_time, u_commanded, y_measured});
    }

    return rows;
}

// Provided -- writes the rows to a CSV in the required format. Do not change.
void writeCsv(const std::string& path, const std::vector<Row>& rows) {
    std::ofstream f(path);
    f << "t,u_commanded,y_measured\n";
    for (const Row& r : rows)
        f << r.t << "," << r.u_commanded << "," << r.y_measured << "\n";
}

// Provided -- runs decodeLog() + writeCsv() for each of the three captures.
int main() {
    const char* names[] = {"step_test", "reversal_test", "deadband_test"};
    for (const char* n : names) {
        const std::string in  = std::string("data/") + n + ".log";
        const std::string out = std::string("data/") + n + ".csv";
        const std::vector<Row> rows = decodeLog(in);
        writeCsv(out, rows);
        std::printf("%-14s %6zu frames -> %s\n", n, rows.size(), out.c_str());
    }
    return 0;
}
