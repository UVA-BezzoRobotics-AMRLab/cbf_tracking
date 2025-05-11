#include "filter/filter_nlopt.h"

#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;
PYBIND11_MODULE(py_filter, m)
{
    py::class_<CBFFilterNLOPT>(m, "CBFFilterNLOPT")
        .def(py::init<>())
        .def("set_dist_map", (bool(CBFFilterNLOPT::*)(py::dict)) &CBFFilterNLOPT::set_dist_map)
        .def("load_params", &CBFFilterNLOPT::load_params)
        .def("Solve", &CBFFilterNLOPT::Solve, "state"_a.noconvert(), "desired_input"_a.noconvert());
}
