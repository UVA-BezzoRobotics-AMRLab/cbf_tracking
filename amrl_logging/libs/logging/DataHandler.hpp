/*
 * @file:    DataHandler.hpp
 * @author:  psherman-uva
 * @date:    Jan 2023
 *
 * @brief: Object to handle multi-threading of
 *         buffering and inserting data into database.
 */

#pragma once

#include <libs/logging/SqliteDatabase.hpp>
#include <libs/logging/SqliteTable.hpp>
#include <libs/threadsafe_queue/ThreadsafeQueue.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <tuple>

namespace amrl {

class DataHandler
{
public:
  /// Constructor
  /// @param  db SQLite database connection object
  DataHandler(std::shared_ptr<SqliteDatabase> db);

  /// Delete copy constructor
  DataHandler(const DataHandler&) = delete;

  /// Delete Assignment operator
  DataHandler operator=(const DataHandler&) = delete;

  /// Destructor
  ~DataHandler(void);

  /// Set the SQLite table to use for logging
  /// @note Will create the table if doesn't exist
  /// @param tbl Sqlite table name to use
  /// @param labels_headers  Header names for label [string] columns
  /// @param integer_headers Header names for integer value columns
  /// @param real_headers    Header names for real value columns
  /// @return True if table is initialized successfully
  bool create_table(const std::string &tbl,
    const std::vector<std::string> &labels_headers,
    const std::vector<std::string> &integer_headers,
    const std::vector<std::string> &real_headers);

  /// Drop table if it exists
  /// @return True if table exists and was able to be dropped from database
  bool drop_table(void);

  /// Insert data into buffer to be inserted into SQLite table
  /// @param labels Labels for next row of data
  /// @param values Next data values to log to table
  void buffer_data(
    const std::vector<std::string> &labels, 
    const std::vector<int> &ints,
    const std::vector<double> &reals);

  /// Check if the data buffer has any remaining data points to insert.
  /// @return True if buffer is empty.
  bool buffer_empty(void);

  /// Get the current size of buffer
  size_t buffer_size(void);

  /// Start-up a thread to begin looping to removing data from
  /// buffer and inserting into the SQLite table
  bool logging_begin(void);

  /// Ends the logging thread
  void logging_end(void);

  /// Check if logging is currently active
  /// @return True if active flag is true
  bool logging_is_active(void);

private:
  /// Internal alias for convenience/readability
  using Row = std::tuple<std::vector<std::string>, std::vector<int>, std::vector<double>>;

  /// Function to run in thread.
  /// Will constantly loop and attempt to
  /// remove data from buffer and insert into database
  void logging_thread_func(void);

  /// Interface object for writing to SQLite database
  std::unique_ptr<SqliteTable> _table;

  /// Threadsafe buffer for data insertion and removed when written to database
  ThreadsafeQueue<Row> _buffer;

  /// Thread object to run logging function
  std::thread _logging_thread;

  /// Flag to signal thread to stop running
  std::atomic<bool> _logging_active;
};

}
