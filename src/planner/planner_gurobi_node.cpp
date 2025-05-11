#include <ros/ros.h>
#include <robust_fast_navigation/planner_gurobi.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "robust_planner");
    ros::NodeHandle nh;

    PlannerROS _planner(nh);

    _planner.spin();

    return 0;
}
