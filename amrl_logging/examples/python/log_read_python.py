#!/usr/bin/env python3

"""
File:   log_read_python.py
Author: psherman-uva
Date:   Jan. 2023

Description: Script to graph data from database
"""


import sqlite3
import matplotlib.pyplot as plt

def read_data_from_database(db_file, table, test):
  sql_cmd = "SELECT * FROM {} WHERE test_name = '{}' ORDER BY timestamp".format(
    table, test, test)

  conn = sqlite3.connect(db_file)
  conn.row_factory = sqlite3.Row
  
  c = conn.cursor()
  c.execute(sql_cmd)

  data = {"timestamp": [], "angle": [], "cosine": [], "sine": []}
  for row in c:
    data["timestamp"].append(row['timestamp'])
    data["cosine"].append(row['cosine'])
    data["sine"].append(row['sine'])
    data["angle"].append(row['angle'])

  conn.close()
  return data


###                      ###
#   Start of Main Script   #
###                      ###

if __name__ == "__main__":
  db_filename = "/home/patrick/uva/database/ProjectDatabase.db"
  table_name  = "ExampleTable"
  test_name   = "python_example"

  data = read_data_from_database(db_filename, table_name, test_name)

  plt.figure(figsize=(10.5, 4.8))
  plt.plot(data['angle'], data['cosine'])
  plt.plot(data['angle'], data['sine'])
  plt.xlabel("radians")
  plt.grid(True)
  plt.tight_layout()

  plt.show()
    
