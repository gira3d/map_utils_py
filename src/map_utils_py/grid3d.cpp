#include <iostream>

#include <map_utils/Grid3DCellValue.h>
#include <map_utils/Grid3D.h>
#include <map_utils/Grid3DParameters.h>
#include <map_utils/Grid3DBoundingBox.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

namespace py = pybind11;
namespace m3 = map_utils::grid3d;
using Grid3D = typename m3::Grid3D<m3::CellValue>;
using Grid3DGrad = typename m3::Grid3D<m3::GradLogodds>;
using Grid3DROI = typename m3::Grid3D<m3::ROILogodds>;

void printParams3D(Grid3D g)
{
  std::cout << "Grid3D Parameters: \n"
            << "\twidth: " << g.width << "\n"
            << "\theight: " << g.height << "\n"
            << "\tdepth: " << g.depth << "\n"
            << "\tresolution: " << g.resolution << "\n"
            << "\tload_time: " << g.load_time << "\n"
            << "\torigin: " << g.origin.x << ", " << g.origin.y << ", "
            << g.origin.z << "\n"
            << "\tlog_odds_hit: " << g.log_odds_hit << "\n"
            << "\tlog_odds_miss: " << g.log_odds_miss << "\n"
            << "\tfree_threshold: " << g.free_threshold << "\n"
            << "\toccupancy_threshold: " << g.occupancy_threshold << "\n"
            << "\tmin_clamp: " << g.min_clamp << "\n"
            << "\tmax_clamp: " << g.max_clamp << "\n"
            << "\tblock_size: " << g.block_size << "\n"
            << "\ttrack_changes: " << g.track_changes << "\n"
            << "\tlock_at_max_clamp: " << g.lock_at_max_clamp << "\n"
            << std::endl;
}

PYBIND11_MODULE(grid3d, m)
{
  m.doc() = "3D occupancy grid via python binding over the map_utils package";

  py::class_<m3::CellValue>(m, "CellValue")
      .def(py::init<>())
      .def(py::init<const float&>())
      .def_readwrite("logodds", &m3::CellValue::logodds)
      .def("__repr__", [](const m3::CellValue& c) {
        return "logodds: " + std::to_string(c.logodds);
      });

  py::class_<m3::GradLogodds, m3::CellValue>(m, "GradLogodds")
      .def(py::init<>())
      .def_readwrite("grad", &m3::GradLogodds::grad)
      .def_readwrite("logodds", &m3::GradLogodds::logodds)
      .def("__repr__", [](const m3::GradLogodds& c) {
        return "logodds: " + std::to_string(c.logodds) +
               " grad x: " + std::to_string(c.grad.x) +
               " grad y: " + std::to_string(c.grad.y) +
               " grad z: " + std::to_string(c.grad.z);
      });

  py::class_<m3::ROILogodds, m3::CellValue>(m, "ROILogodds")
      .def(py::init<>())
      .def_readwrite("roi", &m3::ROILogodds::roi)
      .def_readwrite("distance", &m3::ROILogodds::distance);

  py::enum_<Grid3D::cell_state_t>(m, "CellState")
      .value("FREE", Grid3D::cell_state_t::FREE)
      .value("OCCUPIED", Grid3D::cell_state_t::OCCUPIED)
      .value("UNKNOWN", Grid3D::cell_state_t::UNKNOWN);

  py::class_<m3::Point>(m, "Point")
      .def(py::init<>())
      .def(py::init<float, float, float>())
      .def_readwrite("x", &m3::Point::x)
      .def_readwrite("y", &m3::Point::y)
      .def_readwrite("z", &m3::Point::z)
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self * float())
      .def(py::self / float())
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("__repr__", [](const m3::Point& c) {
        return "x: " + std::to_string(c.x) + " y: " + std::to_string(c.y) +
               " z: " + std::to_string(c.z);
      });

  py::class_<m3::Cell>(m, "Cell")
      .def(py::init<>())
      .def(py::init<unsigned int, unsigned int, unsigned int>())
      .def_readwrite("row", &m3::Cell::row)
      .def_readwrite("col", &m3::Cell::col)
      .def_readwrite("slice", &m3::Cell::slice)
      .def("__repr__", [](const m3::Cell& c) {
        return "r: " + std::to_string(c.row) + " c: " + std::to_string(c.col) +
               " s: " + std::to_string(c.slice);
      });

  py::class_<m3::Parameters>(m, "Parameters")
      .def(py::init<>())
      .def_readwrite("width", &m3::Parameters::width)
      .def_readwrite("height", &m3::Parameters::height)
      .def_readwrite("depth", &m3::Parameters::depth)
      .def_readwrite("resolution", &m3::Parameters::resolution)
      .def_readwrite("origin", &m3::Parameters::origin)
      .def_readwrite("prob_hit", &m3::Parameters::prob_hit)
      .def_readwrite("prob_miss", &m3::Parameters::prob_miss)
      .def_readwrite("free_threshold", &m3::Parameters::free_threshold)
      .def_readwrite("occupancy_threshold",
                     &m3::Parameters::occupancy_threshold)
      .def_readwrite("min_clamp", &m3::Parameters::min_clamp)
      .def_readwrite("max_clamp", &m3::Parameters::max_clamp)
      .def_readwrite("block_size", &m3::Parameters::block_size)
      .def_readwrite("track_changes", &m3::Parameters::track_changes)
      .def_readwrite("lock_at_max_clamp", &m3::Parameters::lock_at_max_clamp);

  py::class_<Grid3D>(m, "Grid3D")
      .def(py::init<>())
      .def(py::init<const m3::Parameters>())
      .def(py::init<Grid3D>())
      .def(py::init<Grid3DGrad>())
      .def("logodds", &Grid3D::logodds)
      .def("get_bbx", &Grid3D::getBBX)
      .def("in_q", py::overload_cast<const m3::Cell&>(&Grid3D::inQ, py::const_))
      .def("in_q",
           py::overload_cast<const m3::Point&>(&Grid3D::inQ, py::const_))
      .def("occupied_q",
           py::overload_cast<unsigned int>(&Grid3D::occupiedQ, py::const_))
      .def("occupied_q",
           py::overload_cast<const m3::Cell&>(&Grid3D::occupiedQ, py::const_))
      .def("occupied_q",
           py::overload_cast<const m3::Point&>(&Grid3D::occupiedQ, py::const_))
      .def("c2w", &Grid3D::c2w)
      .def("w2c", &Grid3D::w2c)
      .def("w2i", &Grid3D::w2i)
      .def("i2w", &Grid3D::i2w)
      .def("probability", &Grid3D::probability)
      .def_readwrite("resolution", &Grid3D::resolution)
      .def_readwrite("width", &Grid3D::width)
      .def_readwrite("height", &Grid3D::height)
      .def_readwrite("depth", &Grid3D::depth)
      .def_readwrite("free_threshold", &Grid3D::free_threshold)
      .def_readwrite("occupancy_threshold", &Grid3D::occupancy_threshold)
      .def_readwrite("min_clamp", &Grid3D::min_clamp)
      .def_readwrite("max_clamp", &Grid3D::max_clamp)
      .def_readwrite("origin", &Grid3D::origin)
      .def("get_ray", py::overload_cast<const m3::Point&, const m3::Point&>(
                          &Grid3D::getRay))
      .def("add_ray",
           py::overload_cast<const m3::Point&, const m3::Point&, float>(
               &Grid3D::addRay))
      .def("set", py::overload_cast<const unsigned int, const m3::CellValue&>(
                      &Grid3D::set))
      .def("set", py::overload_cast<const m3::Cell&, const m3::CellValue&>(
                      &Grid3D::set))
      .def("set", py::overload_cast<const m3::Point&, const m3::CellValue&>(
                      &Grid3D::set))
      .def("get",
           py::overload_cast<const unsigned int>(&Grid3D::get, py::const_))
      .def("get", py::overload_cast<const m3::Cell&>(&Grid3D::get, py::const_))
      .def("get", py::overload_cast<const m3::Point&>(&Grid3D::get, py::const_))
      .def("get_point", py::overload_cast<const unsigned int>(&Grid3D::getPoint, py::const_))
      .def("print_params", &printParams3D);

  py::class_<Grid3DGrad>(m, "Grid3DGrad")
      .def(py::init<>())
      .def(py::init<m3::Parameters>())
      .def(py::init<Grid3D>())
      .def(py::init<Grid3DGrad>())
      .def("logodds", &Grid3DGrad::logodds)
      .def("get_bbx", &Grid3DGrad::getBBX)
      .def("in_q",
           py::overload_cast<const m3::Cell&>(&Grid3DGrad::inQ, py::const_))
      .def("in_q",
           py::overload_cast<const m3::Point&>(&Grid3DGrad::inQ, py::const_))
      .def("occupied_q",
           py::overload_cast<unsigned int>(&Grid3DGrad::occupiedQ, py::const_))
      .def("occupied_q", py::overload_cast<const m3::Cell&>(
                             &Grid3DGrad::occupiedQ, py::const_))
      .def("occupied_q", py::overload_cast<const m3::Point&>(
                             &Grid3DGrad::occupiedQ, py::const_))
      .def("c2w", &Grid3DGrad::c2w)
      .def("w2c", &Grid3DGrad::w2c)
      .def("w2i", &Grid3DGrad::w2i)
      .def("i2w", &Grid3DGrad::i2w)
      .def("probability", &Grid3DGrad::probability)
      .def_readwrite("resolution", &Grid3DGrad::resolution)
      .def_readwrite("width", &Grid3DGrad::width)
      .def_readwrite("height", &Grid3DGrad::height)
      .def_readwrite("depth", &Grid3DGrad::depth)
      .def_readwrite("free_threshold", &Grid3DGrad::free_threshold)
      .def_readwrite("occupancy_threshold", &Grid3DGrad::occupancy_threshold)
      .def_readwrite("min_clamp", &Grid3DGrad::min_clamp)
      .def_readwrite("max_clamp", &Grid3DGrad::max_clamp)
      .def_readwrite("origin", &Grid3DGrad::origin)
      .def("get_ray", py::overload_cast<const m3::Point&, const m3::Point&>(
                          &Grid3DGrad::getRay))
      .def("set", py::overload_cast<const unsigned int, const m3::GradLogodds&>(
                      &Grid3DGrad::set))
      .def("set", py::overload_cast<const m3::Cell&, const m3::GradLogodds&>(
                      &Grid3DGrad::set))
      .def("set", py::overload_cast<const m3::Point&, const m3::GradLogodds&>(
                      &Grid3DGrad::set))
      .def("get",
           py::overload_cast<const unsigned int>(&Grid3DGrad::get, py::const_))
      .def("get",
           py::overload_cast<const m3::Cell&>(&Grid3DGrad::get, py::const_))
      .def("get",
           py::overload_cast<const m3::Point&>(&Grid3DGrad::get, py::const_));

  py::class_<Grid3DROI>(m, "Grid3DROI")
      .def(py::init<>())
      .def(py::init<const m3::Parameters&>())
      .def(py::init<Grid3D>())
      .def(py::init<Grid3DROI>())
      .def("logodds", &Grid3DROI::logodds)
      .def("get_bbx", &Grid3DROI::getBBX)
      .def("in_q",
           py::overload_cast<const m3::Cell&>(&Grid3DROI::inQ, py::const_))
      .def("in_q",
           py::overload_cast<const m3::Point&>(&Grid3DROI::inQ, py::const_))
      .def("occupied_q",
           py::overload_cast<unsigned int>(&Grid3DROI::occupiedQ, py::const_))
      .def("occupied_q", py::overload_cast<const m3::Cell&>(
                             &Grid3DROI::occupiedQ, py::const_))
      .def("occupied_q", py::overload_cast<const m3::Point&>(
                             &Grid3DROI::occupiedQ, py::const_))
      .def("unknown_q",
           py::overload_cast<unsigned int>(&Grid3DROI::unknownQ, py::const_))
      .def("unknown_q", py::overload_cast<const m3::Cell&>(
                             &Grid3DROI::unknownQ, py::const_))
      .def("unknown_q", py::overload_cast<const m3::Point&>(
                             &Grid3DROI::unknownQ, py::const_))
      .def("c2w", &Grid3DROI::c2w)
      .def("w2c", &Grid3DROI::w2c)
      .def("w2i", &Grid3DROI::w2i)
      .def("i2w", &Grid3DROI::i2w)
      .def("probability", &Grid3DROI::probability)
      .def_readwrite("resolution", &Grid3DROI::resolution)
      .def_readwrite("origin", &Grid3DROI::origin)
      .def_readwrite("width", &Grid3DROI::width)
      .def_readwrite("height", &Grid3DROI::height)
      .def_readwrite("depth", &Grid3DROI::depth)
      .def_readwrite("free_threshold", &Grid3DROI::free_threshold)
      .def_readwrite("occupancy_threshold", &Grid3DROI::occupancy_threshold)
      .def("get", py::overload_cast<unsigned int>(&Grid3DROI::get, py::const_))
      .def("get",
           py::overload_cast<const m3::Cell&>(&Grid3DROI::get, py::const_))
      .def("get",
           py::overload_cast<const m3::Point&>(&Grid3DROI::get, py::const_))
      .def("get_index",
           py::overload_cast<const m3::Cell&>(&Grid3DROI::getIndex, py::const_))
      .def("get_index",
           py::overload_cast<const m3::Point&>(&Grid3DROI::getIndex, py::const_))
      .def("set", py::overload_cast<unsigned int, const m3::ROILogodds&>(
                      &Grid3DROI::set))
      .def("set", py::overload_cast<const m3::Cell&, const m3::ROILogodds&>(
                      &Grid3DROI::set))
      .def("set", py::overload_cast<const m3::Point&, const m3::ROILogodds&>(
                      &Grid3DROI::set))
      .def("get_ray", py::overload_cast<const m3::Point&, const m3::Point&>(
                          &Grid3DROI::getRay))
      .def("add_ray",
           py::overload_cast<const m3::Point&, const m3::Point&, float>(
               &Grid3DROI::addRay))
      .def("print_params", &printParams3D);

  py::class_<m3::BoundingBox>(m, "BBox")
      .def(py::init<>())
      .def(py::init<m3::BoundingBox>())
      .def_readwrite("min", &m3::BoundingBox::min)
      .def_readwrite("max", &m3::BoundingBox::max);
}
