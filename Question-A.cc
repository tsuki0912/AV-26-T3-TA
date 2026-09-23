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
#include <iostream>

// One output row.
struct Row {
    double t;            // seconds since the first kept frame
    double u_commanded;  // deg/s
    double y_measured;   // deg
};

// Read the candump log at `path` and return one Row per STEER_ActuatorLog frame, in order.
// Push one Row{t, u_commanded, y_measured} per kept frame.
std::vector<Row> decodeLog(const std::string& path) {
    std::vector<Row> rows;
    std::ifstream file(path);
    int STEER_ActuatorLog_ID = 0x200;

    // same for both 
    int offset = 0;
    double scale = 0.1;

    double first_timestamp = -1;
    // if (!file.is_open()) {
    //     std::cout << "Failed to open file\n";
    //     return 1;
    // }
    std::string line;
    while (std::getline(file, line)) {
        // can_id
        std::string word = "vcan0 ";
        size_t start = line.find(word);
        size_t end = line.find('#');
        int CAN_ID = std::stoi(line.substr(
            start + word.length(), end - (start + word.length())), nullptr, 16
        );
        if (CAN_ID != STEER_ActuatorLog_ID) continue; 
        size_t hash_pos = end;

        // timestamp
        start = line.find('(');
        end = line.find(')');
        double timestamp = std::stod(line.substr(start + 1, end - start - 1));
        if (first_timestamp == -1) {
            first_timestamp = timestamp;
        }
        double t = timestamp - first_timestamp;

        // payload bytes 
        std::string data = line.substr(hash_pos + 1);
        // signed, little endian, 16 bits
        uint8_t byte0 = std::stoul(data.substr(0, 2), nullptr, 16);
        uint8_t byte1 = std::stoul(data.substr(2, 2), nullptr, 16);
        //uint32_t signal_mask = 0x000000FF;

        // extract then combine for little endian
        uint16_t raw_bits = byte0 | (byte1 << 8);
        int16_t MeasuredAngle_raw = static_cast<int16_t>(raw_bits);
        double y_measured = offset + scale * MeasuredAngle_raw;
        
        byte0 = std::stoul(data.substr(4, 2), nullptr, 16);
        byte1 = std::stoul(data.substr(6, 2), nullptr, 16);
        raw_bits = byte0 | (byte1 << 8);
        int16_t CmdAngularRate_raw = static_cast<int16_t>(raw_bits);
        double u_commanded = offset + scale * CmdAngularRate_raw;
        
        rows.push_back({t, u_commanded, y_measured});
    }

    // just returning rows for writeCsv to write
    file.close();
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
