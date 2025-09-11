#!/usr/bin/env python3

import copy

from geometry_msgs.msg import Point

from graph import is_occluded

class PathSmoother():
    def __init__(self, parent_node, graph, alpha, beta):
        self.parent_node_ = parent_node
        self.graph_ = graph
        
        self.alpha_ = alpha
        self.beta_ = beta

        self.path_smooth_ = []

    def smooth_path(self, path_nodes):
        """Smooth the path to remove sharp corners resulting from the grid-based planning"""

        self.parent_node_.get_logger().info('Smoothing path...')

        # Convert into into a geometry_msgs.Point[]
        path = []

        for node in path_nodes:
            p = Point()
            p.x = float(node.x)
            p.y = float(node.y)
            path.append(p)

        # Initialise the smooth path
        path_smooth = copy.deepcopy(path)

        # Loop until the smoothing converges
        # In each iteration, update every waypoint except the first and last waypoint

        ####################
        ## YOUR CODE HERE ##
        ## Task 5         ##
        ####################
        """
        sum_of = 0
        previsous_smooth_path = None
        while True:
            for idx in range(1, len(path_smooth)-1):
                previsous_smooth_path = path_smooth[idx]
                path_smooth[idx] = path_smooth[idx] -(self.alpha_ + 2 * self.beta_) * path_smooth[idx] + self.alpha_ * path[idx] + self.beta_ * (path_smooth[idx-1] + path_smooth[idx+1])
                obstacle = is_occluded(self.graph_.map_.obstacle_map_, [path_smooth[idx-1].x, path_smooth[idx-1].y], [path_smooth[idx].x, path_smooth[idx].y])
                if obstacle:
                    path_smooth[idx] = path[idx]

                sum_of += (path_smooth[idx].x - previsous_smooth_path.y)**2 + (path_smooth[idx].y - previsous_smooth_path.y)**2
                if sum_of < 0.001:
                    self.path_smooth_ = path_smooth
                    break
"""
        
        





        