//PowerCSV.cpp

/*The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
Research and Simulation can be found in README.md in the root directory of
this repository.*/

#include "PowerCSV.h"
#include <iostream>
#include <fstream>
#include <sstream>


/* Function to load pre-generated values as a vector of vectors of doubles
 * Returns std::vector<std::vector<std::double>>
 */
std::vector<std::vector<double>> powerCSV::loadCSV(const std::string& filename){
  ifstream power_csv(filename);
  if (power_csv.fail()) {
    cout << "Unable to open data file" << endl;
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
  return values;
}

