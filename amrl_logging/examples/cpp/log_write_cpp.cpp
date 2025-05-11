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

std::string random_string(int len);
char random_char(void);

// ----------------------------- //
//       MAIN Executable         //
// ----------------------------- //

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "cpp_log_writer");
  ros::NodeHandle nh;

  const std::string table_name("ExampleTable1");
  const std::string topic_name("cpp_example1");
  const std::vector<std::string> label_names({"test_name", "language", "random_str"});
  const std::vector<std::string> int_names({"idx"});
  const std::vector<std::string> value_names({"double", "squared", "negative"});

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
  std::vector<std::string> labels({"write_1", "cpp", ""});
  std::vector<int> ints(1);
  std::vector<double> data(3);

  const double deg2rad = M_PI / 180.0;
  int count = 0;
  while(ros::ok() && count < 500) {
    labels[2] = random_string(5);

    // Include an int
    ints[0] = count;

    // Populate vector with data to log
    size_t idx = 0;
    data[idx++] = count*2.0;
    data[idx++] = pow(count, 2);
    data[idx++] = -1.0*count;

    // Fill out ROS message to publish
    row.header.seq  += 1;
    row.header.stamp = ros::Time::now();
    row.labels       = labels;
    row.nums         = ints;
    row.reals        = data;

    pub.publish(row);
    ros::spinOnce();

    // Sleep until next cycle
    rate.sleep();
    if(++count % 50 == 0) {
      ROS_INFO("Example Loop: Cycle %d", count);
    }
  }
}

std::string random_string(int len)
{
  std::string rs("");
  for (size_t i = 0; i < len; ++i) {
    rs += random_char();
  }
  return rs;
}

char random_char(void) 
{
  static unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  static std::default_random_engine generator(seed);
  static std::uniform_int_distribution<char> distribution(48, 122);

  char ch = distribution(generator);
  if((ch >= 58 && ch <= 64) || (ch >= 91 && ch <= 96)) {
    ch = random_char();
  }
  return ch;
}