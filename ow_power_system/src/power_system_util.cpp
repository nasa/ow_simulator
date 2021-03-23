// TODO (JW): By its nature, this file is designed to be copied. It should
// be explicitely (un)licensed as public domain or CC0.
#include <chrono>
#include <fstream>
#include <sstream>

#include "power_system_util.h"

using namespace std;
using namespace PCOE;

// This function is a quick and dirty reader for the example data files.
// For production-ready applications, a complete and well-tested CSV library
// should be used.
vector<map<MessageId, Datum<double>>> read_file(const string& filename) {
    using namespace chrono;

    ifstream file(filename);
    if (file.fail()) {
        cerr << "Unable to open data file" << endl;
    }
    // Skip header line
    file.ignore(numeric_limits<streamsize>::max(), '\n');

    auto now = system_clock::now();

    vector<map<MessageId, Datum<double>>> result;
    while (file.good()) {
        map<MessageId, Datum<double>> data;
        string line;
        getline(file, line);
        if (line.empty()) {
            continue;
        }
        stringstream line_stream(line);
        string cell;
        getline(line_stream, cell, ',');
        double file_time = stod(cell);
        auto timestamp = now + milliseconds(static_cast<unsigned>(file_time * 1000));

        getline(line_stream, cell, ',');
        Datum<double> power(stod(cell));
        power.setTime(timestamp);

        getline(line_stream, cell, ',');
        Datum<double> temperature(stod(cell));
        temperature.setTime(timestamp);

        getline(line_stream, cell, ',');
        Datum<double> voltage(stod(cell));
        voltage.setTime(timestamp);

        data.insert({MessageId::Watts, power});
        data.insert({MessageId::Centigrade, temperature});
        data.insert({MessageId::Volts, voltage});
        result.push_back(data);
    }
    return result;
}