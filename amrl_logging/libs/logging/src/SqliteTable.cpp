/*
 * File:    SqliteTable.cpp
 * Author:  psherman-uva
 * Date:    Jan. 2023
 */

#include <libs/logging/SqliteTable.hpp>

namespace amrl {

SqliteTable::SqliteTable(std::shared_ptr<SqliteDatabase> database)
  : _db(std::move(database)), _insert_stmt(nullptr), _table_name()
{
}

bool SqliteTable::initialize_table(const std::string &table_name,
    const std::vector<std::string> &labels_headers,
    const std::vector<std::string> &integer_headers,
    const std::vector<std::string> &real_headers)
{
  _insert_stmt = nullptr;
  _table_name = table_name;

  if(!_table_name.empty()) {
    _insert_stmt = _db->create_insert_stmt(_table_name, labels_headers, integer_headers, real_headers);
    return _insert_stmt != nullptr;
  }
  return false;
}

bool SqliteTable::insert_row(
    const std::vector<std::string> &labels, 
    const std::vector<int> &int_values,
    const std::vector<double> &real_values)
{
  if(!_insert_stmt) { return false; }

  size_t idx = 0;
  for(size_t i = 0; i < labels.size(); ++i) { 
    std::string s = ":x" + std::to_string(idx++);
    sqlite3_bind_text(_insert_stmt.get(),
                      sqlite3_bind_parameter_index(_insert_stmt.get(), s.c_str()),
                      labels[i].c_str(),
                      labels[i].size(),
                      SQLITE_STATIC);
  }

  for(size_t i = 0; i < int_values.size(); ++i) {
    std::string s = ":x" + std::to_string(idx++);
    sqlite3_bind_int64(_insert_stmt.get(),
                        sqlite3_bind_parameter_index(_insert_stmt.get(),
                        s.c_str()), 
                        int_values[i]);
  }

  for(size_t i = 0; i < real_values.size(); ++i) {
    std::string s = ":x" + std::to_string(idx++);
    sqlite3_bind_double(_insert_stmt.get(),
                        sqlite3_bind_parameter_index(_insert_stmt.get(),
                        s.c_str()), 
                        real_values[i]);
  }

  bool result = (sqlite3_step(_insert_stmt.get()) == SQLITE_DONE);
  sqlite3_clear_bindings(_insert_stmt.get());
  sqlite3_reset(_insert_stmt.get());
  return result;
}

bool SqliteTable::drop_table(void)
{
  return ready_to_insert() ? _db->delete_table(_table_name) : false;
}

bool SqliteTable::ready_to_insert(void) const
{
  return _insert_stmt != nullptr;
}

std::string SqliteTable::get_table_name(void) const
{
  return _table_name;
}

} // namespace amrl
