#include <iostream>

#include <pybind11_utils/pybind11_utils.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <map_utils/Grid2DROS.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
namespace m2 = map_utils::grid2d;
using Grid2D = typename m2::Grid2D<m2::CellValue>;
using Grid2DGrad = typename m2::Grid2D<m2::GradLogodds>;

nav_msgs::OccupancyGrid toOccupancyGrid(const Grid2D& g,
                                        const std::string& s)
{
  nav_msgs::OccupancyGrid occ_grid = *m2::ros::toOccupancyGrid(g, s);
  return occ_grid;
}

sensor_msgs::PointCloud toPointCloud(const Grid2DGrad& g,
                                     const std::string& s)
{
  sensor_msgs::PointCloud ret = *m2::ros::toPointCloud(g, s);
  return ret;
}

PYBIND11_MODULE(grid2d_ros, m)
{
  m.def("to_occ_grid", &toOccupancyGrid);
  m.def("to_point_cloud", &toPointCloud);
}
