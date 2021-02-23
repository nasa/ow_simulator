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

vector<vector<double>> load_csv(const string& filename){
  ifstream power_csv(filename);
  if (power_csv.fail()) {
    cerr << "Loading power csv file has failed. power_system_node will publish zeros." << endl;
    vector<double> zero_row = {0.0, 0.0, 0.0};
    vector<vector<double>> zeros;
    zeros.push_back(zero_row);
    return zeros;
  }
  vector<vector<double>> values;

  int numColumns = 3;
  //timestamp, voltage, rul for ####watt.csv
  //timestamp, voltage, current for 1HzVoltageProfile.csv
  
  string line,value_string;
  
  double myval;
  vector<double> vec_row;

  //clear first line (header)
  if (power_csv.good()){
    getline(power_csv,line);
  }
  
  // Iterate and grab the values that we want
  while(power_csv.good()){
    vec_row.clear();
    getline(power_csv,line);
    if (line.empty()) {
      continue;
    }
    stringstream line_stream(line);
    //get values out of row, put them in my vector row
    for (int i=0; i < numColumns; i++) {
      getline(line_stream,value_string,',');
      myval = stod(value_string);
      vec_row.push_back(myval);
	
    }
    values.push_back(vec_row);
  }

  power_csv.close();
  cout << "Power csv loaded successfully" << endl;
  return values;
}