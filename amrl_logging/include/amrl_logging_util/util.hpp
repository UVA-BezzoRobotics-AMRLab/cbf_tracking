/*
  File: util.hpp
Author: psherman-uva
  Date: March 2023

Description: Collection of simple useful utility functions to help use AMR logging package
*/

#pragma once

#include <ros/service_client.h>
#include <ros/ros.h>

namespace amrl {

/// Will run intitial setup actions to setup:
/// @param nh ROS node handle object for advertising
/// @param table_name Name of table in database where data will be logged
/// @param topic_name Name of ROS topic streaming to stream data for logging
/// @param label_headers Column names for data labels in table
/// @param real_headers  Column names for data labels in table
/// @param integer_names Column names for data valules in table
/// @return True if calls to setup logging service succeeded
bool logging_setup(ros::NodeHandle &nh, 
  const std::string &table_name, 
  const std::string &topic_name,
  const std::vector<std::string> &label_headers,
  const std::vector<std::string> &integer_headers,
  const std::vector<std::string> &real_headers);

/// Check the amount of rows left in the buffer still left to be written to database
/// @param nh ROS node hangle object
/// @param table_name Name of the table to check
/// @return Number of rows left in buffer
uint32_t logging_buffer_count(ros::NodeHandle &nh, const std::string &table_name);

/// Wait until all data has been logged to the database and then close connection
/// to the table in the database
/// @param nh ROS node handle object
/// @param table_name Name of table to wait until finished
void logging_finish(ros::NodeHandle &nh, const std::string &table_name);

/// Delete a table in the database
/// @param nh ROS node handle object
/// @param table_name Name of table to delete from database
/// @return True if table was successfully removed from database
bool logging_delete_table(ros::NodeHandle &nh, const std::string &table_name);

/// Attempts to read from a database given a select statement
/// @param  db_filename Name (full path) to database
/// @param  select_stmt SQLite select statement
/// @return All rows returned from select statement
std::map<std::string, std::vector<double>> read_data_from_db(
  const std::string &database_file,
  const std::string &select_stmt);

} // namespace amrl
