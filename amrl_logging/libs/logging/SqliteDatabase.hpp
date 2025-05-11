/*
 * @file:   SqliteDatabase.hpp
 * @author: psherman-uva
 * @date:   Jan. 2023
 * 
 * @brief:  Class to hold connection to database and handle table writes
*/

#pragma once

#include <sqlite3.h>
#include <memory>
#include <string>
#include <vector>

namespace amrl {

class SqliteDatabase {
public:
  ///  Constructor
  SqliteDatabase(void);

  /// Delete Copy Constructor
  SqliteDatabase(const SqliteDatabase&) = delete;

  /// Delete assignment operator
  SqliteDatabase& operator=(const SqliteDatabase&) = delete;

  /// Destructor
  ~SqliteDatabase(void);

  /// Open database.
  /// @param  database_file Filename of database
  /// @note Empty string will create a temporary database that is destroyed at object destruction
  /// @return True if database created and/or opened successfully
  bool open_database(const std::string &database_file = "");

  /// Creates (if needed) a table and sqlite statement to insert data into the database
  /// @note Class assumes that first column will be of type STRING and all others REAL
  /// @param  table_name Name of the table to insert data
  /// @param  label_names Names of the labels columns for table
  /// @param  value_names Names of the data value columns for table
  /// @return Valid sqlite3 statement for inserting values into database (NULL if any failures)
  std::shared_ptr<sqlite3_stmt> create_insert_stmt(
    const std::string &table_name,
    const std::vector<std::string> &label_headers,
    const std::vector<std::string> &integer_headers,
    const std::vector<std::string> &real_headers);

  /// Delete a table from the database
  /// @param  table_name Name of the table to delete
  /// @return True if deletion successful
  bool delete_table(const std::string &table_name);

private:

  /// Creates a table in the databse if needed.
  /// @param  table_name Name of the table to insert data
  /// @param  labels Names of the labels columns for table
  /// @param  values Names of the data value columns for table
  bool create_table(const std::string &table_name, 
    const std::vector<std::string> &labels,
    const std::vector<std::string> &integers,
    const std::vector<std::string> &reals);

  /// Pointer to database connection. Should only be one instance in application
  std::shared_ptr<sqlite3> _db;

  /// Directory for database file
  static const std::string kDatabaseDirectory;
};

} // namespace amrl
