/*
 * @file:    log_write_cpp.cpp
 * @author:  psherman-uva
 * @date:    Jan. 2023
 * 
 * @brief: Run example to log data to SQLite node via c++
 */

#include <amrl_logging/LoggingData.h>
#include <amrl_logging/LoggingStart.h>
#include <amrl_logging/LoggingStop.h>
#include <amrl_logging/LoggingDropTable.h>
#include <amrl_logging/LoggingBufferCheck.h>

#include <include/amrl_logging_util/util.hpp>

#include <ros/ros.h>
#include <string>
#include <random>
#include <vector>
#include <cmath>

// ---------------------------------- //
//    Local Function Declarations     //
// ---------------------------------- //

void run_example(ros::NodeHandle &nh, const std::string &topic_name);

// ----------------------------- //
//       MAIN Executable         //
// ----------------------------- //

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cpp_log_writer_2");
  ros::NodeHandle nh;

  const std::string table_name("ExampleTable2");
  const std::string topic_name("cpp_example2");
  const std::vector<std::string> label_names({"test_name", "language"});
  const std::vector<std::string> int_names({});
  const std::vector<std::string> value_names({"angle", "sine", "cosine"});

  if (amrl::logging_delete_table(nh, table_name)
        && amrl::logging_setup(nh, table_name, topic_name, label_names, int_names, value_names))
  {
    run_example(nh, topic_name);
    ROS_INFO("Data send complete");
    
    amrl::logging_finish(nh, table_name);
    ROS_INFO("Test script finished");
  }
  
  return 0;
}

// ---------------------------------- //
//     Local Function Declarations    //
// ---------------------------------- //

void run_example(ros::NodeHandle &nh, const std::string &topic_name)
{
  ros::Publisher pub = nh.advertise<amrl_logging::LoggingData>(topic_name, 100);

  ROS_INFO("Beginning SQL client test.");
  ros::Rate rate(25);

  // Empiraclly found a short pause is needed before logging
  for(size_t i = 0; i < 10; ++i)
    rate.sleep();

  amrl_logging::LoggingData row;
  std::vector<std::string> labels({"write_2", "c++"});
  std::vector<double> data(3);

  const double deg2rad = M_PI / 180.0;
  int count = 0;
  while(ros::ok() && count < 500) {
    // Populate vector with data to log
    size_t idx = 0;
    data[idx++] = count * deg2rad; // Angle in radians
    data[idx++] = sin(data[0]);
    data[idx++] = cos(data[0]);

    // Fill out ROS message to publish
    row.header.seq  += 1;
    row.header.stamp = ros::Time::now();
    row.labels       = labels;
    row.reals        = data;

    pub.publish(row);
    ros::spinOnce();

    // Sleep until next cycle
    rate.sleep();
    if(++count % 50 == 0) {
      ROS_INFO("Example_2 Loop: Cycle %d", count);
    }
  }
}
