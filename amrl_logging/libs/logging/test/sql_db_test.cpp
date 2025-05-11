/*
 * File:   sql_db_test.cpp
 * Author: psherman-uva
 * Date:   Jan. 2023
 *
 * Description: Test script for SQLite library
*/

#include <libs/logging/SqliteDatabase.hpp>
#include <libs/logging/SqliteTable.hpp>

#include <iostream>

using namespace amrl;

int main(int argc, char *argv[])
{
  std::shared_ptr<SqliteDatabase> db = std::make_shared<SqliteDatabase>();
  const std::string dbfile = "/home/patrick/Desktop/test-db-1.db";

  if(db->open_database(dbfile)){
    std::cout << "Database Connection is Open" << std::endl;

    SqliteTable table(db);
    std::string tbl_name = "SqlDbTest";
    std::vector<std::string> col_labels({"TestName", "TrialNum"});
    std::vector<std::string> col_ints({"C"});
    std::vector<std::string> cols_reals({"A", "B", "D"});

    if(table.initialize_table(tbl_name, col_labels, col_ints, cols_reals)) {
      std::cout << "Table is Ready"  << std::endl;

      std::vector<std::string> labels = {"sql_db_test", ""};
      std::vector<double> data_real(3);
      std::vector<int> data_int(1);

      for(int ii = 0; ii < 50; ++ii) {
        labels[1] = std::string("num-") + std::to_string(ii);
        data_int[0]  = 35;
        data_real[0] = ii;
        data_real[1] = ii * 3;
        data_real[2] = ii * ii;

        table.insert_row(labels, data_int, data_real);
      }
    }
  }

  std::cout << "...End of Test" << std::endl;

  return 0;
}
