#include <iostream>

#include <map_utils/Grid2D.h>
#include <map_utils/Grid2DBoundingBox.h>
#include <map_utils/Grid2DCellValue.h>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>

namespace py = pybind11;
namespace m2 = map_utils::grid2d;
using Point = m2::Point;
using Grid2D = m2::Grid2D<m2::CellValue>;
using Grid2DGrad = m2::Grid2D<m2::GradLogodds>;
using Grid2DROI = m2::Grid2D<m2::ROILogodds>;

void printParams2D(Grid2D g)
{
  std::cout << "Grid2D Parameters: \n"
            << "\twidth: " << g.width << "\n"
            << "\theight: " << g.height << "\n"
            << "\tresolution: " << g.resolution << "\n"
            << "\tload_time: " << g.load_time << "\n"
            << "\torigin: " << g.origin.x << ", " << g.origin.y << "\n"
            << "\tlog_odds_hit: " << g.log_odds_hit << "\n"
            << "\tlog_odds_miss: " << g.log_odds_miss << "\n"
            << "\tfree_threshold: " << g.free_threshold << "\n"
            << "\toccupancy_threshold: " << g.occupancy_threshold << "\n"
            << "\tmin_clamp: " << g.min_clamp << "\n"
            << "\tmax_clamp: " << g.max_clamp << "\n"
            << "\tblock_size: " << g.block_size << "\n"
            << "\tlock_at_max_clamp: " << g.lock_at_max_clamp << "\n"
            << std::endl;
}

std::pair<bool, std::pair<std::pair<Point, bool>, std::pair<Point, bool>>>
intersect(Grid2D g, Point p1, Point p2)
{
  Point tp1 = p1;
  Point tp2 = p2;
  bool tshortend = false;
  bool tshortst = false;
  float traynorm = 0.0;

  bool ret = g.intersect(tp1, tp2, tshortend, tshortst, traynorm);

  return std::make_pair(ret, std::make_pair(std::make_pair(tp1, tshortst), std::make_pair(tp2, tshortend)));
}

// Von Neumann neighborhood https://en.wikipedia.org/wiki/Von_Neumann_neighborhood
// Corresponds to Manhattan distance r = 1
void applyAtManhattanNeighbors(
    const m2::Cell &source,
    std::function<void(const m2::Cell &, const m2::Cell &)> foo)
{
  // call on row
  m2::Cell neighbor = source;
  for (int ii = -1; ii <= 1; ii += 2)
  {
    neighbor.row = source.row + ii;
    foo(source, neighbor);
  }

  // call on col
  neighbor = source;
  for (int ii = -1; ii <= 1; ii += 2)
  {
    neighbor.col = source.col + ii;
    foo(source, neighbor);
  }
}

// e.g. the definition of a frontier is:
// cell is unknown && has free neighbor
template <typename CELL_VALUE = m2::CellValue>
bool frontierQ(const typename m2::Grid2D<CELL_VALUE> &grid,
               const m2::Cell &cell)
{
  // only unknown cells may be frontiers
  if (!grid.unknownQ(cell))
  {
    return false;
  }

  // if the source has passed the first check,
  // check neighbors for a free cell
  bool has_free_neighbor = false;
  auto check_neighbor = [&grid,
                         &has_free_neighbor](const m2::Cell &source,
                                             const m2::Cell &neighbor)
  {
    if (grid.inQ(neighbor) && grid.freeQ(neighbor))
    {
      has_free_neighbor = true;
    }
  };

  applyAtManhattanNeighbors(cell, check_neighbor);

  return has_free_neighbor;
}

template <typename T>
void binding_generator(py::module &m, std::string &typestr)
{
  using Grid2DClass = m2::Grid2D<T>;

  std::string pyclass_name = std::string("Grid2D") + typestr;
  py::class_<Grid2DClass>(m, pyclass_name.c_str(), py::dynamic_attr())
      .def(py::init<>())
      .def(py::init<const m2::Parameters &>())
      .def(py::init<Grid2DClass>())
      .def(py::init<unsigned int, unsigned int, float, const m2::Point &>())
      .def("logodds", &Grid2DClass::logodds)
      .def("get_bbx", &Grid2DClass::getBBX)
      .def("in_q", py::overload_cast<const m2::Cell &>(&Grid2DClass::inQ, py::const_))
      .def("in_q",
           py::overload_cast<const m2::Point &>(&Grid2DClass::inQ, py::const_))
      .def("occupied_q",
           py::overload_cast<unsigned int>(&Grid2DClass::occupiedQ, py::const_))
      .def("occupied_q",
           py::overload_cast<const m2::Cell &>(&Grid2DClass::occupiedQ, py::const_))
      .def("occupied_q",
           py::overload_cast<const m2::Point &>(&Grid2DClass::occupiedQ, py::const_))
      .def("c2w", &Grid2DClass::c2w)
      .def("w2c", &Grid2DClass::w2c)
      .def("w2i", &Grid2DClass::w2i)
      .def("i2w", &Grid2DClass::i2w)
      .def("probability", &Grid2DClass::probability)
      .def_readwrite("min_clamp", &Grid2DClass::min_clamp)
      .def_readwrite("max_clamp", &Grid2DClass::max_clamp)
      .def_readwrite("resolution", &Grid2DClass::resolution)
      .def_readwrite("origin", &Grid2DClass::origin)
      .def_readwrite("width", &Grid2DClass::width)
      .def_readwrite("height", &Grid2DClass::height)
      .def_readwrite("free_threshold", &Grid2DClass::free_threshold)
      .def_readwrite("occupancy_threshold", &Grid2DClass::occupancy_threshold)
      .def("add_registered_pointcloud", &Grid2DClass::addRegisteredPointCloud)
      .def("get", py::overload_cast<unsigned int>(&Grid2DClass::get, py::const_))
      .def("get", py::overload_cast<const m2::Cell &>(&Grid2DClass::get, py::const_))
      .def("get", py::overload_cast<const m2::Point &>(&Grid2DClass::get, py::const_))
      .def("get_index",
           py::overload_cast<const m2::Cell &>(&Grid2DClass::getIndex, py::const_))
      .def("get_index",
           py::overload_cast<const m2::Point &>(&Grid2DClass::getIndex, py::const_))
      .def("set",
           py::overload_cast<unsigned int, const T &>(&Grid2DClass::set))
      .def("set", py::overload_cast<const m2::Cell &, const T &>(
                      &Grid2DClass::set))
      .def("set", py::overload_cast<const m2::Point &, const T &>(
                      &Grid2DClass::set))
      .def("get_ray", py::overload_cast<const m2::Point &, const m2::Point &>(
                          &Grid2DClass::getRay))
      .def("add_ray",
           py::overload_cast<const m2::Point &, const m2::Point &, float>(
               &Grid2DClass::addRay))
      .def("intersect", &intersect)
      .def("print_params", &printParams2D)
      .def("frontier_q", &frontierQ<T>)
      .def(py::pickle(
          [](const Grid2DClass &g)
          { return py::make_tuple(g.params, g.data); },
          [](py::tuple t)
          {
            Grid2DClass g(t[0].cast<m2::Parameters>());
            g.data = t[1].cast<std::vector<T>>();
            return g;
          }));
  ;
}

PYBIND11_MODULE(grid2d, m)
{
  m.doc() = "2D occupancy grid via python binding over the map_utils package";

  py::class_<m2::BoundingBox>(m, "BBox")
      .def(py::init<>())
      .def(py::init<m2::BoundingBox>())
      .def_readwrite("min", &m2::BoundingBox::min)
      .def_readwrite("max", &m2::BoundingBox::max);

  std::string c1 = "";
  binding_generator<m2::CellValue>(m, c1);

  std::string c2 = "Grad";
  binding_generator<m2::GradLogodds>(m, c2);

  std::string c3 = "ROI";
  binding_generator<m2::ROILogodds>(m, c3);

  py::class_<m2::CellValue>(m, "CellValue")
      .def(py::init<>())
      .def(py::init<const float &>())
      .def_readwrite("logodds", &m2::CellValue::logodds)
      .def("__repr__", [](const m2::CellValue &c)
           { return "logodds: " + std::to_string(c.logodds); })
      .def(py::pickle(
          [](const m2::CellValue &r)
          {
            return py::make_tuple(r.logodds);
          },
          [](py::tuple t)
          {
            m2::CellValue r;
            r.logodds = t[0].cast<float>();
            return r;
          }));

  py::class_<m2::GradLogodds, m2::CellValue>(m, "GradLogodds")
      .def(py::init<>())
      .def_readwrite("grad", &m2::GradLogodds::grad)
      .def("__repr__", [](const m2::GradLogodds &c)
           { return "logodds: " + std::to_string(c.logodds) +
                    " grad x: " + std::to_string(c.grad.x) +
                    " grad y: " + std::to_string(c.grad.y); })
      .def(py::pickle(
          [](const m2::GradLogodds &r)
          {
            return py::make_tuple(r.logodds, r.grad);
          },
          [](py::tuple t)
          {
            m2::GradLogodds r;
            r.logodds = t[0].cast<float>();
            r.grad = t[1].cast<m2::Point>();
            return r;
          }));

  py::class_<m2::ROILogodds, m2::CellValue>(m, "ROILogodds")
      .def(py::init<>())
      .def_readwrite("roi", &m2::ROILogodds::roi)
      .def_readwrite("distance", &m2::ROILogodds::distance)
      .def(py::pickle(
          [](const m2::ROILogodds &r)
          {
            return py::make_tuple(r.logodds, r.roi, r.distance);
          },
          [](py::tuple t)
          {
            m2::ROILogodds r;
            r.logodds = t[0].cast<float>();
            r.roi = t[1].cast<bool>();
            r.distance = t[2].cast<float>();
            return r;
          }));

  py::enum_<Grid2D::cell_state_t>(m, "CellState")
      .value("FREE", Grid2D::cell_state_t::FREE)
      .value("OCCUPIED", Grid2D::cell_state_t::OCCUPIED)
      .value("UNKNOWN", Grid2D::cell_state_t::UNKNOWN);

  py::class_<m2::Point>(m, "Point")
      .def(py::init<>())
      .def(py::init<float, float>())
      .def(py::self + py::self)
      .def(py::self - py::self)
      .def(py::self * float())
      .def(py::self / float())
      .def(py::self += py::self)
      .def(py::self -= py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def_readwrite("x", &m2::Point::x)
      .def_readwrite("y", &m2::Point::y)
      .def("__repr__",
           [](const m2::Point &c)
           {
             return "x: " + std::to_string(c.x) + " y: " + std::to_string(c.y);
           })
      .def(py::pickle(
          [](const m2::Point &p)
          { return py::make_tuple(p.x, p.y); },
          [](py::tuple t)
          {
            m2::Point p(t[0].cast<float>(), t[1].cast<float>());
            return p;
          }));

  py::class_<m2::Cell>(m, "Cell")
      .def(py::init<>())
      .def(py::init<unsigned int, unsigned int>())
      .def_readwrite("row", &m2::Cell::row)
      .def_readwrite("col", &m2::Cell::col)
      .def("__repr__", [](const m2::Cell &c)
           { return "r: " + std::to_string(c.row) + " c: " + std::to_string(c.col); });

  py::class_<m2::Parameters>(m, "Parameters")
      .def(py::init<>())
      .def_readwrite("width", &m2::Parameters::width)
      .def_readwrite("height", &m2::Parameters::height)
      .def_readwrite("resolution", &m2::Parameters::resolution)
      .def_readwrite("origin", &m2::Parameters::origin)
      .def_readwrite("prob_hit", &m2::Parameters::prob_hit)
      .def_readwrite("prob_miss", &m2::Parameters::prob_miss)
      .def_readwrite("free_threshold", &m2::Parameters::free_threshold)
      .def_readwrite("occupancy_threshold",
                     &m2::Parameters::occupancy_threshold)
      .def_readwrite("min_clamp", &m2::Parameters::min_clamp)
      .def_readwrite("max_clamp", &m2::Parameters::max_clamp)
      .def_readwrite("block_size", &m2::Parameters::block_size)
      .def_readwrite("lock_at_max_clamp", &m2::Parameters::lock_at_max_clamp)
      .def(py::pickle(
          [](const m2::Parameters &p)
          {
            return py::make_tuple(
                p.width, p.height, p.resolution, p.origin, p.prob_hit,
                p.prob_miss, p.free_threshold, p.occupancy_threshold,
                p.min_clamp, p.max_clamp, p.block_size, p.lock_at_max_clamp);
          },
          [](py::tuple t)
          {
            m2::Parameters p;
            p.width = t[0].cast<unsigned int>();
            p.height = t[1].cast<unsigned int>();
            p.resolution = t[2].cast<float>();
            p.origin = t[3].cast<m2::Point>();
            p.prob_hit = t[4].cast<float>();
            p.prob_miss = t[5].cast<float>();
            p.free_threshold = t[6].cast<float>();
            p.occupancy_threshold = t[7].cast<float>();
            p.min_clamp = t[8].cast<float>();
            p.max_clamp = t[9].cast<float>();
            p.block_size = t[10].cast<unsigned int>();
            p.lock_at_max_clamp = t[11].cast<bool>();

            return p;
          }));
}
