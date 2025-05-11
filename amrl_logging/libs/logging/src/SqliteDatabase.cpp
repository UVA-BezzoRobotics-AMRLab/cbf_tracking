/*
 * File:    SqliteDatabase.cpp
 * Author:  psherman-uva
 * Date:    Jan. 2023
 */

#include <libs/logging/SqliteDatabase.hpp>

namespace amrl {

SqliteDatabase::SqliteDatabase(void) :
  _db(nullptr)
{
}

SqliteDatabase::~SqliteDatabase(void)
{
  _db.reset();
  sqlite3_shutdown();
}

bool SqliteDatabase::open_database(const std::string &database_file)
{
  // Open in read/write mode, creates database if doesn't currently exist
  static constexpr int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;

  sqlite3 *db_local     = nullptr;
  std::string full_path = database_file.empty() ? "" : database_file;

  int err = sqlite3_open_v2(full_path.c_str(), &db_local, flags, NULL);
  if(err == SQLITE_OK){
    // The following PRAGMA's improve insertion times by ~99%
    // Sacrifices robustness and ability for multiple database connections
    // Okay for a personal project like this where its not a big deal
    sqlite3_exec(db_local, "PRAGMA synchronous = OFF", NULL, NULL, NULL);
    sqlite3_exec(db_local, "PRAGMA journal_mode = OFF", NULL, NULL, NULL);
    sqlite3_exec(db_local, "PRAGMA locking_mode = EXCLUSIVE", NULL, NULL, NULL);

    _db = std::shared_ptr<sqlite3>(db_local, [](sqlite3 *p) { sqlite3_close(p); } );
    return true;
  }

  return false;
}

std::shared_ptr<sqlite3_stmt> SqliteDatabase::create_insert_stmt(
  const std::string &table_name,
    const std::vector<std::string> &label_headers,
    const std::vector<std::string> &integer_headers,
    const std::vector<std::string> &real_headers)
{
  // Make sure table exists
  if (!create_table(table_name, label_headers, integer_headers, real_headers)) { 
    return nullptr; 
  }

  // Create SQLite insert statement
  // INSERT INTO <table-name> (<name-1>, <name-2>, ..., <name-n>) VALUES ( :x1, :x2, ..., :xn)
  std::string cmd = "INSERT INTO " + table_name + " (";
  
  for(const auto &s : label_headers)   { cmd += s + ", "; }
  for(const auto &s : integer_headers) { cmd += s + ", "; }
  for(const auto &s : real_headers)    { cmd += s + ", "; }
  
  cmd.pop_back(); // Remove trailing ", " characters
  cmd.pop_back();
  cmd += ") VALUES (";
  
  size_t N = label_headers.size() + integer_headers.size() + real_headers.size();
  for(size_t i = 0; i < N; ++i) {
    cmd += ":x" + std::to_string(i) + ", ";
  }
  
  cmd.pop_back(); // Remove trailing ", " characters
  cmd.pop_back();
  cmd += ")";

  // Prepare statment object pointer
  sqlite3_stmt* stmt;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
    sqlite3_finalize(stmt);
    return nullptr;
  }

  return std::shared_ptr<sqlite3_stmt>(stmt, [](sqlite3_stmt *p) { sqlite3_finalize(p); });
}

bool SqliteDatabase::create_table(const std::string &table_name, 
    const std::vector<std::string> &labels,
    const std::vector<std::string> &integers,
    const std::vector<std::string> &reals) 
{
  if(!_db || (labels.empty() && integers.empty() && reals.empty())) { return false; }

  std::string cmd = "CREATE TABLE IF NOT EXISTS " + table_name + " (";

  for(const auto &str : labels)   { cmd += str + " STRING, "; }
  for(const auto &str : integers) { cmd += str + " INTEGER, "; }
  for(const auto &str : reals)    { cmd += str + " REAL, "; }

  cmd.pop_back(); // Remove trailing ", " characters
  cmd.pop_back();
  cmd += ")";

  sqlite3_stmt *st = nullptr;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &st, nullptr) != SQLITE_OK) {
    sqlite3_finalize(st);
    return false; //
  }

  bool result = (sqlite3_step(st) == SQLITE_DONE);
  sqlite3_finalize(st);
  return result;
}

bool SqliteDatabase::delete_table(const std::string &table_name)
{
  if(!_db || table_name.empty()) { return false; }

  std::string cmd = "DROP TABLE IF EXISTS " + table_name;
  sqlite3_stmt *st = nullptr;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &st, NULL) != SQLITE_OK) {
    sqlite3_finalize(st);
    return false; // Failed to drop table in SQLite database
  }

  sqlite3_step(st);
  sqlite3_finalize(st);
  return true;
}

} // namespace amrl