#!/usr/bin/env python
import rospkg
import yaml
import rospy
from sensor_msgs.msg import PointCloud2

from grid3d import Grid3D, Parameters
import grid3d_ros

def convert_dict_to_params(d):
  p = Parameters()
  p.width = d['map']['width']
  p.height = d['map']['height']
  p.depth = d['map']['depth']
  p.resolution = d['map']['resolution']
  p.origin.x = d['map']['origin_x']
  p.origin.y = d['map']['origin_y']
  p.origin.z = d['map']['origin_z']
  p.min_clamp = d['map']['clamping_min']
  p.max_clamp = d['map']['clamping_max']
  p.free_threshold = d['map']['free_threshold']
  p.occupancy_threshold = d['map']['occupancy_threshold']
  p.prob_hit = d['map']['probability_hit']
  p.prob_miss = d['map']['probability_miss']
  p.block_size = d['map']['block_size']
  p.lock_at_max_clamp = d['map']['lock_at_max_clamp']
  p.track_changes = d['map']['track_changes']

  return p

def main():
  rp = rospkg.RosPack()
  this_ros_pkg_path = rp.get_path('map_utils_py')
  p = Parameters()
  with open(this_ros_pkg_path + '/config/grid3d.yaml') as file:
    params = yaml.load(file)
    p = convert_dict_to_params(params)
  
  g = Grid3D(p)
  g.print_params()

  og = grid3d_ros.to_unknown_grid(g, "world")

  rospy.init_node("grid3d_ros_test")
  grid_pub = rospy.Publisher("occ_grid", PointCloud2, queue_size=10)
  rate = rospy.Rate(25)

  while not rospy.is_shutdown():
    grid_pub.publish(og)
    rate.sleep()


if __name__ == '__main__':
    main()