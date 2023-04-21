#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from typing import final
import numpy as np
import matplotlib.pyplot as plt
import bresenham as bh
import math

def plot_gridmap(gridmap):
    plt.figure()
    plt.imshow(gridmap, cmap='Greys',vmin=0, vmax=1)
    
def init_gridmap(size, res):
    gridmap = np.zeros([int(np.ceil(size/res)), int(np.ceil(size/res))])
    return gridmap

def world2map(pose, gridmap, map_res):
    origin = np.array(gridmap.shape)/2
    new_pose = np.zeros(pose.shape)
    new_pose[0] = np.round(pose[0]/map_res) + origin[0];
    new_pose[1] = np.round(pose[1]/map_res) + origin[1];
    return new_pose.astype(int)

def v2t(pose):
    c = np.cos(pose[2])
    s = np.sin(pose[2])
    tr = np.array([[c, -s, pose[0]], [s, c, pose[1]], [0, 0, 1]])
    return tr    

def ranges2points(ranges):
    # laser properties
    start_angle = -1.5708
    angular_res = 0.0087270
    max_range = 30
    # rays within range
    num_beams = ranges.shape[0]
    idx = (ranges < max_range) & (ranges > 0)
    # 2D points
    angles = np.linspace(start_angle, start_angle + (num_beams*angular_res), num_beams)[idx]
    points = np.array([np.multiply(ranges[idx], np.cos(angles)), np.multiply(ranges[idx], np.sin(angles))])
    # homogeneous points
    points_hom = np.append(points, np.ones((1, points.shape[1])), axis=0)
    return points_hom

def ranges2cells(r_ranges, w_pose, gridmap, map_res):
    # ranges to points
    r_points = ranges2points(r_ranges)
    w_P = v2t(w_pose)
    w_points = np.matmul(w_P, r_points)
    # covert to map frame
    m_points = world2map(w_points, gridmap, map_res)
    m_points = m_points[0:2,:]
    return m_points

def poses2cells(w_pose, gridmap, map_res):
    # covert to map frame
    m_pose = world2map(w_pose, gridmap, map_res)
    return m_pose  

def bresenham(x0, y0, x1, y1):
    l = np.array(list(bh.bresenham(x0, y0, x1, y1)))
    return l
    
def prob2logodds(prob):
    logodds = math.log((prob)/(1-prob))
    return logodds
    
def logodds2prob(logodds):
    prob = 1 - (1/(1 + math.exp(logodds)))
    return prob

def inv_sensor_model(cell, endpoint, prob_occ, prob_free):
    ray_cast = bresenham(cell[0], cell[1], endpoint[0], endpoint[1])
    prob_vals = []
    for ray in range(len(ray_cast)-1):
        prob_vals.append(prob_free)
    prob_vals.append(prob_occ)
    prob_vals = np.array(prob_vals).reshape((len(ray_cast), 1))
    inv_sensor_model = np.hstack((ray_cast, prob_vals))
    return inv_sensor_model


def grid_mapping_with_known_poses(ranges_raw, poses_raw, occ_gridmap, map_res, prob_occ, prob_free, prior):
  for time_idx in range(len(poses_raw)):
      current_position = poses2cells(poses_raw[time_idx], occ_gridmap, map_res)
      detected_cells = ranges2cells(ranges_raw[time_idx], current_position, occ_gridmap, map_res)
      final_sensor_model = np.array([]).reshape(0,3)
      for detected in range(detected_cells.shape[1]):
          detected_cell = [1,1]
          detected_cell[0] = detected_cells[0][detected]
          detected_cell[1] = detected_cells[1][detected]
          #print("Detected Cell: ", detected_cell[0], ", " , detected_cell[1])
          sensor_model = inv_sensor_model(current_position, detected_cell, prob_occ, prob_free) 
          final_sensor_model = np.concatenate((final_sensor_model, sensor_model), axis=0)
      final_sensor_model = np.unique(final_sensor_model, axis=0) 

      for cell_to_update in final_sensor_model:
          print(cell_to_update)
          #occ_gridmap[int(cell_to_update[0]), int(cell_to_update[1])] += cell_to_update[2]
      plot_gridmap(occ_gridmap)    


           



    

    #   for row in occ_gridmap:
    #       for cell in row:
               #if cell is in perceptual field:
               
               


