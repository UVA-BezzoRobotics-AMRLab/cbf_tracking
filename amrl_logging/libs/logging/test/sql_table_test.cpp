/*
 * File:   sql_table_test.cpp
 * Author: psherman-uva
 * Date:   Jan 2023
 *
 * Description: Test script for SQLite library
*/

#include <libs/logging/SqliteTable.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <atomic>
#include <thread>
#include <random>

using namespace amrl;

static const std::string db_filename = "/home/patrick/Desktop/TestDatabase.db";

void write_to_table(std::unique_ptr<SqliteTable> tbl)
{
  static int s = 0;
  int val = (s++);
  std::vector<std::string> tst = {"thread_test", "thread"+std::to_string(val)};
  std::vector<int>    nums(2);
  std::vector<double> data(3);

  for(int ii = 0; ii < 10000; ++ii) {
    nums[0] = 45;
    nums[1] = 51;

    data[0] = val*ii;
    data[1] = ii * 3;
    data[2] = ii * ii;
    tbl->insert_row(tst, nums, data);
  }
}

int main(int argc, char* argv[])
{
  std::shared_ptr<SqliteDatabase> db = std::make_shared<SqliteDatabase>();
  if(db->open_database(db_filename)) {
    std::unique_ptr<SqliteTable> tbl1 = std::unique_ptr<SqliteTable>(new  SqliteTable(db));
    std::unique_ptr<SqliteTable> tbl2 = std::unique_ptr<SqliteTable>(new  SqliteTable(db));
    std::unique_ptr<SqliteTable> tbl3 = std::unique_ptr<SqliteTable>(new  SqliteTable(db));

    std::string tbl_name1 = "Table01";
    std::string tbl_name2 = "Table02";
    std::string tbl_name3 = "Table03";
    std::vector<std::string> labels({"test", "label"});
    std::vector<std::string> ints({"D", "F"});
    std::vector<std::string> cols({"A", "B", "C"});

    if(tbl1->initialize_table(tbl_name1, labels,  ints, cols) &&
        tbl2->initialize_table(tbl_name2, labels, ints, cols) &&
        tbl3->initialize_table(tbl_name3, labels, ints, cols)) {

      // Create three threads
      // Insert a bunch of data at the same time
      std::thread t1 = std::thread(write_to_table, std::move(tbl1));
      std::thread t2 = std::thread(write_to_table, std::move(tbl2));
      std::thread t3 = std::thread(write_to_table, std::move(tbl3));

      if(t1.joinable())
        t1.join();
      if(t2.joinable())
        t2.join();
      if(t3.joinable())
        t3.join();
    }
  } else {
    std::cout << "Unable to open database: " << db_filename << std::endl;
  }

  std::cout << "...End of Test" << std::endl;
  return 0;
}
