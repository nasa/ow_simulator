// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// OW power system ROS node - publishes 0 as a placeholder for once Battery models are linked.

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>
//CSV handling
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>

using namespace std;

vector<vector<double>> loadCSV(const string& filename);
//void computeSOC(const std_msgs::Float64& power){} // placeholder for calculating SOC with battery model
//double getSampleValue(double timestamp, vector<vector<double>> &power_csv); //output for testing with precalculated csv data
//std::vector<std::vector<std::double>>> loadCSV(string csvName);

int main(int argc, char* argv[]) {

  //bool useSampleValues = true;
  
  // if (useSampleValues){
  // }
  
  ros::init(argc,argv,"power_system_node");
  ros::NodeHandle n ("power_system_node");

  //Construct our power draw listener
  //TODO: Check out latching
  
  //ros::Subscriber power_listener = n.subscribe("power_draw", 1000, computeSOC);

  //Construct our State of Charge (SOC) publisher
  ros::Publisher SOC_pub = n.advertise<std_msgs::Float64>("state_of_charge",1000);

  //Define our publication rate and initialize our time to 0
  double power_update_rate = 1.0;  // 1 Hz, tailor as needed
  double timestamp = 0.0;

  //Load power values csv
  string csv_path;
  csv_path = ros::package::getPath("ow_power_system");
  csv_path += "/data/1HzVoltageProfile.csv";
  vector<vector<double>> power_csv = loadCSV(csv_path);
  cout << "I loaded!";

  int voltage_i = 1; //index of voltage column
  int minutes_to_skip = 30; //enables us to skip first 30 minutes with no power being drawn
  int t = minutes_to_skip * 60;
  
  // ROS Loop. Note that once this loop starts,
  // this function (and node) is terminated with an interrupt.
  
  ros::Rate rate(power_update_rate);
  //individual soc_msg to be published by SOC_pub
  std_msgs::Float64 soc_msg;
  ROS_INFO ("Power system node running");
  
  while (ros::ok()) {

    soc_msg.data=power_csv[t][voltage_i];
    //soc_msg.data=t;
    //soc_msg.data=getSampleValue(timestamp, power_csv); //zero for testing purposes, will be calculated later

    //publish current SOC
    SOC_pub.publish(soc_msg);
    t += 1.0;
    
    ros::spinOnce();
    rate.sleep();
   
  }

  return 0;
}


// function to load pre-generated values as a vector of vectors of floats
vector<vector<double>> loadCSV(const string& filename){
  ifstream power_csv(filename);
  //power_csv.open(filename,std::ios::in);
  if (power_csv.fail()) {
    cout << "Unable to open data file" << endl;
    //cerr << "Unable to open data file" << endl;
  }
  vector<vector<double>> values;

  int numColumns = 3;//timestamp, voltage, current for 1HzVoltageProfile.csv
  //power_csv.ignore(numeric_limits<streamsize>::max(),'\n'); //throw away first line if column headings are present
  
  string line,value_string;
  
  double myval;
  vector<double> vec_row; 
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

// convert timestamp (seconds) to closest value from preloaded power csv, return power value
/*double getSampleValue(double timestamp, vector<vector<double>> &power_csv){
  
  //convert power node time stamp to correct value
  double csv_timestep = 0.003;
  
  //grab SOC (voltage) from csv
  //

  return 0.0;
  }*/
