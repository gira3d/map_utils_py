#!/usr/bin/env python
import yaml
from yaml import Loader

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

import os
from grid2d import Grid2D, Parameters, Point

def convert_dict_to_params(d):
  p = Parameters()
  p.width = d['map']['width']
  p.height = d['map']['height']
  p.resolution = d['map']['resolution']
  p.origin.x = d['map']['origin_x']
  p.origin.y = d['map']['origin_y']
  p.min_clamp = d['map']['clamping_min']
  p.max_clamp = d['map']['clamping_max']
  p.free_threshold = d['map']['free_threshold']
  p.occupancy_threshold = d['map']['occupancy_threshold']
  p.prob_hit = d['map']['probability_hit']
  p.prob_miss = d['map']['probability_miss']
  p.block_size = d['map']['block_size']
  p.lock_at_max_clamp = d['map']['lock_at_max_clamp']

  return p

def test_intersect(g):
  bbx = g.get_bbx()

  #define Matplotlib figure and axis
  fig, ax = plt.subplots()

  #create simple line plot
  ax.plot([bbx.min.x, bbx.min.x],[bbx.min.y, bbx.max.y], 'k')
  ax.plot([bbx.min.x, bbx.max.x],[bbx.min.y, bbx.min.y], 'k')
  ax.plot([bbx.max.x, bbx.max.x],[bbx.min.y, bbx.max.y], 'k')
  ax.plot([bbx.min.x, bbx.max.x],[bbx.max.y, bbx.max.y], 'k')

  ax.plot([-20,5],[-20,5], 'r')
  ret = g.intersect(Point(-20,-20), Point(5,5))
  if ret[0]:
    new_min = ret[1][0][0]
    new_max = ret[1][1][0]
    ax.plot([new_min.x, new_max.x], [new_min.y, new_max.y], 'b')

  #display plot
  plt.show()

def test_add_registered_pcld(g):
  pass

def main():
  script_path = os.getcwd()
  param_path = script_path + "/../config/grid2d.yaml"

  p = Parameters()
  with open(param_path) as file:
    params = yaml.load(file, Loader=Loader)
    p = convert_dict_to_params(params)

  g = Grid2D(p)
  g.print_params()

  test_intersect(g)
  #test_add_registered_pcld(g):

if __name__ == '__main__':
  main()
