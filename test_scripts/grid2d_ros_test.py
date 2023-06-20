#!/usr/bin/env python
import rospkg
import yaml
import rospy
from nav_msgs.msg import OccupancyGrid

from grid2d import Grid2D, Parameters, Point
import grid2d_ros

from utils import convert_dict_to_params



def main():
  
  rp = rospkg.RosPack()
  this_ros_pkg_path = rp.get_path('map_utils_py')
  p = Parameters()
  with open(this_ros_pkg_path + '/config/grid2d.yaml') as file:
    params = yaml.load(file)
    p = convert_dict_to_params(params)
  
  # build grid from params
  g = Grid2D(p)
  g.print_params()
  
  # query point and change to occupied
  query_point = Point(1.0,1.0)

  g.set(query_point, g.logodds(1.0))

  # read map point probability
  point_logodds = g.get(query_point)
  print("point_probability=",g.probability(point_logodds))
  point_state = g.occupiedQ(query_point)
  print("point_state=",point_state)

  # initialize start/end point and ray trace
  st = Point(0.0,0.0)
  en = Point(2.0,2.0)

  success, raycells = g.get_ray(st, en)
  if (success):
    for r in raycells:
      print((g.c2w(r)).x, (g.c2w(r)).y )

  # transform to occupancy grid and publish to ros
  og = grid2d_ros.to_occ_grid(g, "world")

  rospy.init_node("grid2d_ros_test")
  grid_pub = rospy.Publisher("occ_grid", OccupancyGrid, queue_size=10)
  rate = rospy.Rate(25)

  while not rospy.is_shutdown():
    grid_pub.publish(og)
    rate.sleep()



if __name__ == '__main__':
  main()