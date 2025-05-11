#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "mpc/mpc_core.h"

namespace py = pybind11;

PYBIND11_MODULE(py_mpc, m)
{
    py::enum_<mpc_type>(m, "mpc_type")
        .value("MPC_TYPE_CTE", MPC_TYPE_CTE)
        .value("MPC_TYPE_ACC", MPC_TYPE_ACC)
        .value("MPC_TYPE_NLOPT", MPC_TYPE_NLOPT)
        .value("MPC_TYPE_NLOPT_POS", MPC_TYPE_NLOPT_POS)
        .export_values();

    py::class_<JackalMPCCore>(m, "JackalMPCCore")
        .def(py::init<>())
        .def(py::init<const mpc_type &>())
        .def("load_params", &JackalMPCCore::load_params)
        .def("set_odom", (void(JackalMPCCore::*)(const Eigen::Vector3d &)) & JackalMPCCore::set_odom)
        .def("set_odom", (void(JackalMPCCore::*)(const std::vector<double> &)) & JackalMPCCore::set_odom)
        .def("set_goal", (void(JackalMPCCore::*)(const Eigen::Vector2d &)) & JackalMPCCore::set_goal)
        .def("set_goal", (void(JackalMPCCore::*)(const std::vector<double> &)) & JackalMPCCore::set_goal)
        .def("set_obstacle", (void(JackalMPCCore::*)(const Eigen::Vector3d &)) & JackalMPCCore::set_obstacle)
        .def("set_obstacle", (void(JackalMPCCore::*)(const std::vector<double> &)) & JackalMPCCore::set_obstacle)
        .def("set_reference", (void(JackalMPCCore::*)(const Eigen::MatrixXd &)) & JackalMPCCore::set_reference)
        .def("set_reference", (void(JackalMPCCore::*)(const std::vector<double> &)) & JackalMPCCore::set_reference)
        .def("get_mpc_results", &JackalMPCCore::get_mpc_results)
        .def("set_dist_map", (bool(JackalMPCCore::*)(py::dict grid_dict)) & JackalMPCCore::set_dist_map)
        .def("set_dist_map", (void(JackalMPCCore::*)(const std::shared_ptr<distmap::DistanceMap> &dist_map)) & JackalMPCCore::set_dist_map)
        .def("get_dist_map_data", &JackalMPCCore::get_dist_map_data)
        .def("set_mpc_type", &JackalMPCCore::set_mpc_type)
        .def("getHorizon", &JackalMPCCore::getHorizon)
        .def("solve", &JackalMPCCore::solve)
        .def("evaluateHFunction", &JackalMPCCore::evaluateHFunction);
}
