from grid2d import Parameters

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