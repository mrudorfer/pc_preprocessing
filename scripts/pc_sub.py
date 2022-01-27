#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np


def pc2_to_numpy(point_cloud2_msg):
    data = ros_numpy.numpify(point_cloud2_msg)
    data = data.flatten()
    for col in ['x', 'y', 'z']:
        isnan = np.isnan(data[col])
        data = data[~isnan]

    pc = np.empty((len(data), 3), dtype=np.float32)
    pc[:, 0] = data['x']
    pc[:, 1] = data['y']
    pc[:, 2] = data['z']
    return pc


def crop_point_cloud(pc, lower_limits=None, upper_limits=None):
    """ discards all points that are not within limits """
    if lower_limits is None:
        lower_limits = [-1, -1, 0]
    if upper_limits is None:
        upper_limits = [1, 1, 2]
    assert len(lower_limits) == len(upper_limits) == 3, f'length of limits vectors must be three, ' \
                                                        f'got {len(lower_limits)} and {len(upper_limits)}'
    for i in range(len(lower_limits)):
        assert lower_limits[i] < upper_limits[i], f'lower limit must be lower than upper limit. ' \
                                                  f'this is not the case for {["x", "y", "z"][i]} ({i})'
        mask = lower_limits[i] < pc[:, i]
        mask &= pc[:, i] < upper_limits[i]
        pc = pc[mask]
    return pc


class PointCloudSubscriber:
    def __init__(self):
        rospy.init_node('pc_sub', anonymous=True)
        self.input = rospy.get_param('~input', '/preprocess/outlier_removal/output')
        self.filename = rospy.get_param('~filename', '/home/rudorfem/ros/example_pc.npy')
        rospy.Subscriber(self.input, PointCloud2, self.callback)

        print('initialising PointCloudSubscriber with following parameters:')
        for key, val in vars(self).items():
            print(f'\t{key}: {val}')

        self.latest_point_cloud = None
        self.save_data_flag = False

        while not rospy.is_shutdown():
            key = input()
            if key in ['q', 'quit', 'exit']:
                break
            self.save_data_flag = True

    def callback(self, data):
        self.latest_point_cloud = pc2_to_numpy(data)
        print('pc:', self.latest_point_cloud.shape)
        if self.save_data_flag:
            np.save(self.filename, self.latest_point_cloud)
            print(f'saved latest point cloud to {self.filename}')
            self.save_data_flag = False


if __name__ == '__main__':
    try:
        PointCloudSubscriber()
    except rospy.ROSInterruptException:
        print('PointCloudSubscriber got interrupted...')
