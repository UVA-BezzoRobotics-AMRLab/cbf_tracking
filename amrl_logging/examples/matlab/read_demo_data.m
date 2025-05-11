%{
  File: read_demo_data.m
Author: psherman-uva
  Date: Feb. 2023

Description: Example MATLAB script for reading data from a SQLite database
with data written by other exmple writing scripts for package demos.

Note: Requires Database Toolbox
%}

clear all; close; clc;
set(0,'DefaultAxesFontSize', 24,'DefaultLineLineWidth', 4); % Sets default font size for all figures


db_filename = "/home/patrick/Desktop/GroupDemo.db"; % Filename for local database
table_name  = "RosbotData"; % Name of table in database where example data is stored

conn = sqlite(db_filename, "readonly");
data = sqlread(conn, table_name);
close(conn);

idx = strcmp(data.shape, "square") & strcmp(data.trial, "trial-1");
data_sqaure1 = data(idx, :);

figure();
plot(data_sqaure1.vicon_x, data_sqaure1.vicon_y, 'bo'); hold on;
plot(data_sqaure1.pose_x, data_sqaure1.pose_y, 'ro');


