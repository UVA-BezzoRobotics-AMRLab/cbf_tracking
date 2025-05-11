
#include <amrl_logging/LoggingData.h>
#include <amrl_logging/LoggingStart.h>
#include <amrl_logging/LoggingStop.h>
#include <amrl_logging/LoggingDropTable.h>
#include <amrl_logging/LoggingBufferCheck.h>

#include <include/amrl_logging_util/util.hpp>

#include <sqlite3.h>

namespace amrl {

static std::shared_ptr<sqlite3> open_database(const std::string &db_filename)
{
  static constexpr int flags = SQLITE_OPEN_READONLY;

  sqlite3 *db_local = nullptr;
  int err = sqlite3_open_v2(db_filename.c_str(), &db_local, flags, nullptr);

  if(err == SQLITE_OK){
    std::shared_ptr<sqlite3> db = std::shared_ptr<sqlite3>(db_local, [](sqlite3 *p) { sqlite3_close(p); });
    return db;
  }

  return nullptr;
}

bool logging_setup(ros::NodeHandle &nh, 
  const std::string &table_name,
  const std::string &topic_name, 
  const std::vector<std::string> &label_headers,
  const std::vector<std::string> &integer_headers,
  const std::vector<std::string> &real_headers)
{
  /// Checks if service exists as a way to see if the logging node is active
  ros::ServiceClient sql_client = nh.serviceClient<amrl_logging::LoggingStart>("/amrl_logger/start_log");
  
  if(sql_client.waitForExistence(ros::Duration(5.0))) {
    amrl_logging::LoggingStart start_log;

    start_log.request.table_name  = table_name;
    start_log.request.topic_name  = topic_name;
    
    start_log.request.label_headers   = label_headers;
    start_log.request.integer_headers = integer_headers;
    start_log.request.real_headers    = real_headers;

    if (sql_client.call(start_log)) {
      if (start_log.response.success) {
        ROS_INFO("Logging service setup succeeded");
        return true;
      } else {
        ROS_ERROR("Logging startup service failed setup");
        return false;
      }
    } else {
      ROS_ERROR("Call to logging startup service failed");
      return false;
    }
  }

  ROS_ERROR("Logging startup service unreachable");
  return false;
}

uint32_t logging_buffer_count(ros::NodeHandle &nh, const std::string &table_name)
{
  ros::ServiceClient client_buffer_check = nh.serviceClient<amrl_logging::LoggingBufferCheck>("/amrl_logger/log_buffer_check");
  if(client_buffer_check.exists()) {
    amrl_logging::LoggingBufferCheck buffer_check;
    buffer_check.request.table_name = table_name;

    if(client_buffer_check.call(buffer_check)) {
      return buffer_check.response.size;
    } else {
      ROS_WARN("Logging queue check server unreachable.");
    }
  }
  
  return 0;
}

void logging_finish(ros::NodeHandle &nh, const std::string &table_name)
{
  // Logging node reports how many messages are left in queue to be inserted into database
  // When number is zero, we know all data has been inserted into table
  ros::ServiceClient client_logging_check = nh.serviceClient<amrl_logging::LoggingBufferCheck>("/amrl_logger/log_buffer_check");
  while(client_logging_check.exists()) {
    amrl_logging::LoggingBufferCheck logging_done;
    logging_done.request.table_name = table_name;

    if(client_logging_check.call(logging_done)) {
      ros::Duration(0.2).sleep();
      if(logging_done.response.is_empty && logging_done.response.size == 0) { break; }
    } else {
      ROS_WARN("Logging queue check server unreachable.");
      break;
    }
  }
  ROS_INFO("Logging queue is Empty");

  // Tell logging node to stop listening to topic and remove and
  ros::ServiceClient sql_client = nh.serviceClient<amrl_logging::LoggingStop>("/amrl_logger/stop_log");
  if(sql_client.waitForExistence(ros::Duration(5.0))) {
    amrl_logging::LoggingStop stop_log;
    stop_log.request.table_name = table_name;
    sql_client.call(stop_log);
  } else {
    ROS_WARN("Stop logging server unreachable.");
  }
}

bool logging_delete_table(ros::NodeHandle &nh, const std::string &table_name)
{
  std::string client_name = "/amrl_logger/drop_table";

  if(ros::service::waitForService(client_name, ros::Duration(5.0))) {
    ros::ServiceClient client = nh.serviceClient<amrl_logging::LoggingDropTable>(client_name);  
    amrl_logging::LoggingDropTable data;
    data.request.table_name = table_name;

    client.call(data);
    // if(data.response.table_existed)
    //   ROS_INFO("Table to Drop %s existed", table_name.c_str());
    // else
    //   ROS_INFO("Table to Drop %s didn't existed", table_name.c_str());
    
    return data.response.success;
  }
  
  ROS_WARN("Service: %s timeout on wait", client_name.c_str());
  return false;
}

std::map<std::string, std::vector<double>> read_data_from_db(
  const std::string &database_file,
  const std::string &select_stmt)
{
  std::map<std::string, std::vector<double>> data;

  std::shared_ptr<sqlite3> db = open_database(database_file);
  if(db) {
    sqlite3_stmt* stmt;
    if(sqlite3_prepare_v2(db.get(), select_stmt.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {
      bool first  = true;
      int num_col = 0;

      while(sqlite3_step(stmt) == SQLITE_ROW) {
        if(first) { // First loop, initialize map with column names and empty vector
          num_col = sqlite3_column_count(stmt);
          for (int i = 0; i < num_col; ++i) {
            std::string col_name = std::string(sqlite3_column_name(stmt, i));
            data[col_name]       = std::vector<double>();
          }
          first = false;
        }

        for(int i = 0; i < num_col; ++i) {
          std::string col_name = std::string(sqlite3_column_name(stmt, i));
          double val = sqlite3_column_double(stmt, i);
          data[col_name].push_back(val);
        }
      }
    }
    
    // Always finalize, even if failed to prepare
    sqlite3_finalize(stmt);
  } else {
    std::cout << "Failed to open database: " << database_file << std::endl;
  }

  return data;
}

} // namespace amrl
