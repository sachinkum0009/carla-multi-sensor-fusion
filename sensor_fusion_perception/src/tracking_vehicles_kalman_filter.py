#!/usr/bin/env python3

'''
Tracking of the vehicle using the kalman filter


Author: Sachin Kumar
Date: 08-02-2024
'''

import rclpy
from rclpy.node import Node
import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class TrackVehicleKalman(Node):
    def __init__(self):
        super().__init__(self, 'track_vehicle_kalman')
        self.timer = self.create_timer(0.33, self.timer_callback)
        '''
        dim x how many states you have (disp, velocity)
        dim z how many input you have
        '''
        self.kalman_filter = KalmanFilter(dim_x=2, dim_z=1)

    def timer_callback(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = TrackVehicleKalman()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

