/*
 * @file:    sqlite_node.cpp
 * @author:  psherman-uva
 * @date:    Jan. 2023
 * 
 * @brief: Program to run SQLite logging ROS node 
 */

#include <libs/logging/SqliteTable.hpp>
#include <libs/logging/SqliteDatabase.hpp>
#include <libs/logging/DataHandler.hpp>

#include <amrl_logging/LoggingData.h>
#include <amrl_logging/LoggingStart.h>
#include <amrl_logging/LoggingStop.h>
#include <amrl_logging/LoggingDropTable.h>
#include <amrl_logging/LoggingBufferCheck.h>

#include <ros/ros.h>
#include <boost/function.hpp>
#include <string>

#include <iostream>

using namespace amrl;

// ---------------------------------- //
//   Callback Function Declarations   //
// ---------------------------------- //

// Function for LoggingStart service.
// Creates a new SQLite table object and will subscribe to topic to insert into database
bool logging_start(
  amrl_logging::LoggingStart::Request &req,
  amrl_logging::LoggingStart::Response &res,
  ros::NodeHandle &nh);

// Function for LoggingStop service.
// Creates a new SQLite table object and will subscribe to topic to insert into database
bool logging_stop(amrl_logging::LoggingStop::Request &req, amrl_logging::LoggingStop::Response &res);

/// Function for LoggingDropTable service.
/// Stops logging if active, then drops table from SQLite database
bool logging_drop_table(amrl_logging::LoggingDropTable::Request &req, amrl_logging::LoggingDropTable::Response &res);

/// Function for LoggingBufferCheck service.
/// Checks if logging has any buffered data to insert into database.
bool logging_buff_check(amrl_logging::LoggingBufferCheck::Request &req, amrl_logging::LoggingBufferCheck::Response &res);

/// Callback function for the ROS subscriber.
/// Saves msg for logging into SQLite table
/// @param  msg Published datapoint
/// @param  logger Database object to log msg to
void logging_buffer_msg(const amrl_logging::LoggingData::ConstPtr &msg, std::shared_ptr<amrl::DataHandler> logger);

// ----------------------------- //
//      Static Variables         //
// ----------------------------- //

static std::shared_ptr<SqliteDatabase> db_connection = nullptr;
static std::map<std::string, std::pair<std::shared_ptr<amrl::DataHandler>, ros::Subscriber>> data_handlers;

// ----------------------------- //1
//       MAIN Executable         //
// ----------------------------- //

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "logging_node");
  ros::NodeHandle nh("amrl_logger");

  std::string db_filename;
  if(!nh.getParam("/logging_node/db_filename", db_filename)) { 
    ROS_ERROR("Required paramter \'db_filename\' not set"); 
    return 0;
  }

  /// Should only be a single instance of this object running at a time
  db_connection = std::make_shared<SqliteDatabase>();
  if(db_connection->open_database(db_filename)) {
    boost::function<bool(amrl_logging::LoggingStart::Request&, amrl_logging::LoggingStart::Response&)> logging_start_cb = 
      boost::bind(logging_start, _1, _2, nh);

    // Start node services
    ros::ServiceServer start_server = nh.advertiseService("start_log", logging_start_cb);
    ros::ServiceServer stop_server  = nh.advertiseService("stop_log", logging_stop);
    ros::ServiceServer buffer_check_server = nh.advertiseService("log_buffer_check", logging_buff_check);
    ros::ServiceServer remove_table_server = nh.advertiseService("drop_table", logging_drop_table);

    // Start continuos ROS Loop
    ROS_INFO("SQLite Logging Node Started with Database file: %s", db_filename.c_str());
    if(ros::ok()) { 
      ros::spin(); 
    }
  } else {
    ROS_ERROR("SQLite logging node unable to open database file: %s", db_filename.c_str());
  }
  
  return 0;
}

// ---------------------------------- //
//   Callback Function Declarations   //
// ---------------------------------- //

void logging_buffer_msg(const amrl_logging::LoggingData::ConstPtr &msg, std::shared_ptr<amrl::DataHandler> logger)
{
  logger->buffer_data(msg->labels, msg->nums, msg->reals);
}

bool logging_start(
    amrl_logging::LoggingStart::Request &req,
    amrl_logging::LoggingStart::Response &res,
    ros::NodeHandle &nh)
{
  data_handlers[req.table_name] = std::pair<std::shared_ptr<amrl::DataHandler>, ros::Subscriber>();
  data_handlers[req.table_name].first = std::make_shared<amrl::DataHandler>(db_connection);

  // Try to create table in the database with desired column names
  if(data_handlers[req.table_name].first->create_table(req.table_name, req.label_headers, req.integer_headers, req.real_headers)){
    // Callback for messages
    boost::function<void(const amrl_logging::LoggingData::ConstPtr &)>
    msg_callback = boost::bind(logging_buffer_msg, _1, data_handlers[req.table_name].first);

    // Subscribe to topic
    std::string topic = req.topic_name;
    if(topic.length() > 0 && topic[0] != '/') { topic.insert(0, "/"); }
    data_handlers[req.table_name].second = nh.subscribe(topic, 100, msg_callback);

    // Log Success
    res.success = data_handlers[req.table_name].first->logging_begin();;
    ROS_INFO("SQLite logging started to table: %s", req.table_name.c_str());
    ROS_INFO("Subscribing to: %s", topic.c_str());
    return true;
  } else {
    data_handlers.erase(req.table_name);
    ROS_WARN("Failed to open DB or create table: %s", req.table_name.c_str());
  }

  res.success = false;
  return false;
}

bool logging_stop(amrl_logging::LoggingStop::Request &req, amrl_logging::LoggingStop::Response &res)
{
  if(data_handlers.find(req.table_name) == data_handlers.end()) {
    ROS_WARN("SQLite logging stop for invalid table name: %s", req.table_name.c_str());
    return false;
  }

  if(data_handlers[req.table_name].first->logging_is_active()) {
    data_handlers[req.table_name].first->logging_end();
    while(data_handlers[req.table_name].first->logging_is_active()) { ros::Duration(0.2).sleep(); }
  }

  ROS_INFO("SQLite logging stopped for table: %s", req.table_name.c_str());
  return true;
}

bool logging_drop_table(amrl_logging::LoggingDropTable::Request &req, amrl_logging::LoggingDropTable::Response &res)
{
  res.success = false;

  if(data_handlers.find(req.table_name) != data_handlers.end() &&
        data_handlers[req.table_name].first->logging_is_active()) {
    data_handlers[req.table_name].first->logging_end();
    while(data_handlers[req.table_name].first->logging_is_active()) { ros::Duration(0.2).sleep(); }
    data_handlers[req.table_name].first = nullptr;
  }

  if(db_connection->delete_table(req.table_name)) {
    res.success = true;
    ROS_INFO("SQLite Table %s Dropped.", req.table_name.c_str());
    return true;
  }

  ROS_INFO("SQLite failed to drop table: %s.", req.table_name.c_str());
  return false;
}

bool logging_buff_check(amrl_logging::LoggingBufferCheck::Request &req, amrl_logging::LoggingBufferCheck::Response &res)
{
  if(data_handlers.find(req.table_name) != data_handlers.end()) {
    res.size     = data_handlers[req.table_name].first->buffer_size();
    res.is_empty = data_handlers[req.table_name].first->buffer_empty();
    return true;
  }

  ROS_WARN("SQLite buffer check called with invalid table name: %s", req.table_name.c_str());
  res.size     = 0;
  res.is_empty = false;
  return false;
}
