#include <iostream>

#include <pybind11_utils/pybind11_utils.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <map_utils/Grid3DCellValue.h>
#include <map_utils/Grid3DROS.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
namespace m3 = map_utils::grid3d;
using Grid3D = typename m3::Grid3D<m3::CellValue>;
using Grid3DGrad = typename m3::Grid3D<m3::GradLogodds>;

static bool publish_greater_than_zero(double value) { return value > 1e-3; }

sensor_msgs::PointCloud2
toSensorMsgsPointCloud2(const sensor_msgs::PointCloud &p) {
  sensor_msgs::PointCloud2 p2;
  sensor_msgs::convertPointCloudToPointCloud2(p, p2);
  return p2;
}

sensor_msgs::PointCloud2 toOccupiedPointCloud(const Grid3D &g,
                                              const std::string &s) {
  sensor_msgs::PointCloud pts = *m3::ros::toOccupiedPointCloud(g, s);
  return toSensorMsgsPointCloud2(pts);
}

sensor_msgs::PointCloud2 toUnknownPointCloud(const Grid3D &g,
                                             const std::string &s) {
  sensor_msgs::PointCloud pts = *m3::ros::toUnknownPointCloud(g, s);
  return toSensorMsgsPointCloud2(pts);
}

sensor_msgs::PointCloud2 toFreePointCloud(const Grid3D &g,
                                          const std::string &s) {
  sensor_msgs::PointCloud pts = *m3::ros::toFreePointCloud(g, s);
  return toSensorMsgsPointCloud2(pts);
}

sensor_msgs::PointCloud toDistancePointCloud(const Grid3DGrad &g,
                                             const std::string &s) {
  sensor_msgs::PointCloud distance_grid;
  distance_grid =
      *m3::ros::toPointCloud<m3::GradLogodds, publish_greater_than_zero>(g, s);
  return distance_grid;
}

PYBIND11_MODULE(grid3d_ros, m) {
  m.def("to_occupied_grid", &toOccupiedPointCloud);
  m.def("to_unknown_grid", &toUnknownPointCloud);
  m.def("to_free_grid", &toFreePointCloud);
  m.def("to_distance_grid", &toDistancePointCloud);
}
