from __future__ import absolute_import
from . import sense
import logging
import rospy
import actionlib
import message_filters
from sensor_msgs.msg import CameraInfo, Image as ImageSensor_msg
from dope_msgs.msg import DopeAction, DopeGoal


class DopeSensor(sense.BaseSensor):
    def __init__(self):
        super(DopeSensor, self).__init__()

        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Action client
        self._goal = None
        self._client = actionlib.SimpleActionClient('dope', DopeAction)

        # Start ROS subscribers
        image_sub = message_filters.Subscriber(
            '/dope/webcam/image_raw',
            ImageSensor_msg
        )
        info_sub = message_filters.Subscriber(
            '/dope/webcam/camera_info',
            CameraInfo
        )
        ts = message_filters.TimeSynchronizer([image_sub, info_sub], 1)
        ts.registerCallback(self.image_callback)

    def image_callback(self, image, info):
        self._goal = DopeGoal()
        self._goal.image = image
        self._goal.cam_info = info

    def process_data(self, data):
        self._client.send_goal(self._goal)
        self._client.wait_for_result()
        result = self._client.get_result()
        self._logger.debug('Result is {}'.format(result))
        return None


sense.sensors['dope_sensor'] = DopeSensor
