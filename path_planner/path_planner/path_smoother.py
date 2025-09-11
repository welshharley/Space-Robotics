#!/usr/bin/env python3

import copy

from geometry_msgs.msg import Point

from .graph import is_occluded

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
        
        
        for _ in range(1000):
            sum_of = 0.0
            for idx in range(1, len(path_smooth)-1):
                new_x = path_smooth[idx].x - (self.alpha_ + 2 * self.beta_) * path_smooth[idx].x + self.alpha_ * path[idx].x + self.beta_ * (path_smooth[idx-1].x + path_smooth[idx+1].x)
                new_y = path_smooth[idx].y - (self.alpha_ + 2 * self.beta_) * path_smooth[idx].y + self.alpha_ * path[idx].y + self.beta_ * (path_smooth[idx-1].y + path_smooth[idx+1].y)
                
                block_prev = is_occluded(self.graph_.map_.obstacle_map_, [path_smooth[idx-1].x, path_smooth[idx-1].y], [new_x, new_y])
                block_next = is_occluded(self.graph_.map_.obstacle_map_, [new_x, new_y], [path_smooth[idx+1].x, path_smooth[idx+1].y])
                if block_next or block_prev:
                   continue


                dx = new_x - path_smooth[idx].x
                dy = new_y - path_smooth[idx].y
                sum_of += dx * dx + dy *dy

                path_smooth[idx].x = new_x
                path_smooth[idx].y = new_y

            if sum_of < 0.001:
                break

        self.path_smooth_ = path_smooth



        