// OW power system ROS node - publishes CSV values as a placeholder for once Battery models are linked.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__


//ROS
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
void computeSOC(const std_msgs::Float64& power){} // placeholder for calculating SOC with battery model
double getSampleValue(double timestamp, vector<vector<double>> &power_csv); //output for testing with precalculated csv data
//std::vector<std::vector<std::double>>> loadCSV(string csvName);

int main(int argc, char* argv[]) {

  //bool useSampleValues = true;
  
  // if (useSampleValues){
    //load csv up - If we do it here, we only have to do it once
    //std::vector<std::vector<std::float>>> powerValues;
    //powerValues = loadCSV();
  // }
  
  ros::init(argc,argv,"power_system_node");
  ros::NodeHandle n ("power_system_node");

  //Construct our power draw listener
  //TODO: Check out latching
  
  ros::Subscriber power_listener = n.subscribe("power_draw", 1000, computeSOC);

  //Construct our State of Charge (SOC) publisher
  ros::Publisher SOC_pub = n.advertise<std_msgs::Float64>("state_of_charge",1000);

  //Define our publication rate and initialize our time to 0
  double power_update_rate = 1.0;  // 1 Hz, tailor as needed
  double timestamp = 0.0;

  //Load power values csv
  vector<vector<double>> power_csv = loadCSV("Voltage_Profile1.csv");
  cout << "I loaded!";
  double t = 0;
  int voltage_i = 2;
  
  // ROS Loop. Note that once this loop starts,
  // this function (and node) is terminated with an interrupt.
  
  ros::Rate rate(power_update_rate);
  while (ros::ok()) {
    //individual soc_msg to be published by SOC_pub
    std_msgs::Float64 soc_msg;

    soc_msg.data=power_csv[0][voltage_i];
    //soc_msg.data=t;
    //soc_msg.data=getSampleValue(timestamp, power_csv); //zero for testing purposes, will be calculated later

    //publish current SOC
    SOC_pub.publish(soc_msg);
    //t += 1.0;
    
    ros::spinOnce();
    rate.sleep();
    ROS_INFO ("Power system node running");
  }

  return 0;
}


// function to load pre-generated values as a vector of vectors of floats
vector<vector<double>> loadCSV(const string& filename){
  ifstream power_csv(filename);
  //power_csv.open(filename,std::ios::in);
  if (power_csv.fail()) {
    cout << "Unable to open data file" << endl;
    cerr << "Unable to open data file" << endl;
  }
  vector<vector<double>> values;

  int numColumns = 4;//timestep, current, voltage, timestamp
  //power_csv.getline(row,100,'\n'); //throw away first line
  power_csv.ignore(numeric_limits<streamsize>::max(),'\n');
  
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
double getSampleValue(double timestamp, vector<vector<double>> &power_csv){
  
  //convert power node time stamp to correct value
  double csv_timestep = 0.003;
  
  //grab SOC (voltage) from csv
  //

  return 0.0;
}
