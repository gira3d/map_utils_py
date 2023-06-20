#!/usr/bin/env python
import yaml

from grid3d import Grid3D, Grid3DGrad, CellValue, GradLogodds, Parameters

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
  with open('../config/grid3d.yaml') as file:
    params = yaml.safe_load(file)
    p = convert_dict_to_params(params)

  cell_val = CellValue()
  cell_val.logodds = 4
  g = Grid3D(p)
  g.print_params()
  g.set(15, cell_val);
  cell_val2 = g.get(15);

  g2 = Grid3DGrad(g)
  grad_logodds = g2.get(15)
  print("Output should be 4.0 and it is " + str(grad_logodds.logodds))
  grad_logodds = g2.get(16)
  print("Output should be 0.0 and it is " + str(grad_logodds.logodds))

if __name__ == '__main__':
    main()
