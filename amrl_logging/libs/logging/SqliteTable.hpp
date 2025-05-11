/*
 * @file:   SqliteTable.hpp
 * @author: psherman-uva
 * @date:   Jan. 2023
 * 
 * @brief:  Class for logging data with SQLite library to a single table.
 */

#pragma once

#include <libs/logging/SqliteDatabase.hpp>

#include <sqlite3.h>
#include <string>
#include <memory>
#include <vector>

namespace amrl {

class SqliteTable 
{
public:

  /// Constructor
  /// @param  database_file Filename of SQLite database
  SqliteTable(std::shared_ptr<SqliteDatabase> database);

  /// Destructor
  ~SqliteTable(void) = default;

  /// Makes sure the table exists in the database connection and unlocks
  /// ability for object to insert data into databse
  /// @param table_name
  /// @param labels_headers
  /// @param integer_headers
  /// @param real_headers
  bool initialize_table(const std::string &table_name, 
    const std::vector<std::string> &labels_headers,
    const std::vector<std::string> &integer_headers,
    const std::vector<std::string> &real_headers);

  /// Insert row into table
  /// @param  labels
  /// @param  data
  /// @return True if row inserted into table successfully
  bool insert_row(
    const std::vector<std::string> &labels, 
    const std::vector<int> &int_values,
    const std::vector<double> &real_values);

  /// Check if table has been initialized and ready to insert rows into database
  /// @return True if valid insert statement has been created
  bool ready_to_insert(void) const;

  /// Drop table from the database
  /// @return True if table was able to be dropped
  bool drop_table(void);

  /// @return Get the the table name
  std::string get_table_name(void) const;

private:
  /// Pointer to database
  std::shared_ptr<SqliteDatabase> _db;

  /// Insertion statement to persists for life of logging process
  std::shared_ptr<sqlite3_stmt> _insert_stmt;

  /// Name of table in database
  std::string _table_name;

  /// Directory for database
  static const std::string kDatabaseDirectory;
};

} // namespace amrl
