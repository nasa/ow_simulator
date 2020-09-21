//PowerCSV.h

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

class PowerCSV {
  public:
    vector<vector<double>> load(const string& filename);
    int getTimeIndex;
    int getVoltageIndex;

  private:
    int time_i;

}
