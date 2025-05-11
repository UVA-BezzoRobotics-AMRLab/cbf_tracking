/*
 * @file:   log_read_cpp.cpp
 * @author: psherman-uva
 * @date:   Jan. 2023
 * 
 * @brief: Example reading data from database
*/

#include <include/amrl_logging_util/util.hpp>

#include <sqlite3.h>

#include <map>
#include <vector>
#include <string>
#include <memory>
#include <iostream>
#include <algorithm>
#include <cstdio>


int main(int argc, char *argv[])
{
  const std::string db_filename = "/home/patrick/uva/database/ProjectDatabase.db";
  const std::string table_name  = "ExampleTable";
  const std::string test_name   = "cpp_example";

  const std::string cmd = "SELECT * FROM " + table_name +
  " WHERE test_name = '" + test_name + "' ORDER BY timestamp";

  auto data = amrl::read_data_from_db(db_filename, cmd);
  
  std::vector<std::string> columns({
    "timestamp",
    "angle",
    "cosine",
    "sine",
    "tangent"});

  if(!std::all_of(
    columns.begin(), columns.end(),
    [&data](std::string s) { return data.find(s) != data.end(); })) 
  {
    std::cerr << "Not all expected columns found in database" << std::endl;
    return -1;
  }


  // Print table of data, one row at a time.
  char buffer[100];
  std::sprintf(buffer, "%12s %10s %10s %10s %10s\n", "timestamp", "angle", "cosine", "sine", "tangent");
  std::cout << buffer;
  
  for (int i = 0; i < data["timestamp"].size(); ++i) {
    double time_stamp  = data["timestamp"][i];
    double angle = data["angle"][i];
    double cos = data["cosine"][i];
    double sin = data["sine"][i];
    double tan = data["tangent"][i];

    std::sprintf(buffer, "%12.1f %10.3f %10.3f %10.3f %10.3f\n", time_stamp, angle, cos, sin, tan);
    std::cout << buffer;
  }

  return 0;
}
