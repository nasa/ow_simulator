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
  ros::Publisher RUL_pub = n.advertise<std_msgs::Float64>("remaining_useful_life",1000);
  
  //Load power values csv
  string csv_path,csv_file;
  csv_file = "/data/onewatt.csv";
  //FILE OPTIONS: onewatt.csv, eightwatt.csv, sixteenwatt.csv
  csv_path = ros::package::getPath("ow_power_system");
  csv_path += csv_file;
  
  vector<vector<double>> power_csv = loadCSV(csv_path);
  cout << "Power system CSV loaded!";
  
  //Define our publication rate and initialize our time to 0
  double power_update_rate;  
  if (csv_file=="/data/onewatt.csv"){
    power_update_rate = 0.10; //csv is at 10Hz intervals
  } else {
    power_update_rate = 1.0; // 1 Hz default
  }
  int t_step= 1;//starting at 1 not zero to skip csv headings

  //set our indices
  int time_i = 0;
  int voltage_i = 1; //index of voltage column
  int rul_i = 2;
  
  // ROS Loop. Note that once this loop starts,
  // this function (and node) is terminated with an interrupt.
  
  ros::Rate rate(power_update_rate);
  //individual soc_msg to be published by SOC_pub
  std_msgs::Float64 soc_msg;
  std_msgs::Float64 rul_msg;
  ROS_INFO ("Power system node running");

  int power_lifetime = power_csv.size();
  while (ros::ok()) {
    if (t_step < power_lifetime) {
      soc_msg.data=power_csv[t_step][voltage_i];
      rul_msg.data=power_csv[t_step][rul_i];
    } //should keep publishing last voltage value if sim time exceeds the length of the csv
    
    //soc_msg.data=t;
    //soc_msg.data=getSampleValue(timestamp, power_csv); //zero for testing purposes, will be calculated later

    //publish current SOC
    SOC_pub.publish(soc_msg);
    RUL_pub.publish(rul_msg);
    t_step+=1;
    
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

// convert timestamp (seconds) to closest value from preloaded power csv, return power value
/*double getSampleValue(double timestamp, vector<vector<double>> &power_csv){
  
  //convert power node time stamp to correct value
  double csv_timestep = 0.003;
  
  //grab SOC (voltage) from csv
  //

  return 0.0;
  }*/
