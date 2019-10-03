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

    def get_next_image(self, timeout=5):
        # Reset goal
        self._goal = None

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

        # Wait for message with timeout
        start = rospy.get_rostime()
        timeout = rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while (self._goal is None and (not rospy.is_shutdown()) and
                                      (timeout == 0 or rospy.get_rostime() - start < timeout)):
            rate.sleep()

        if self._goal is None:
            self._logger.error('Failed to get an image within {}s'.format(timeout.to_sec()))

        # Unregister image subscription
        image_sub.unregister()
        info_sub.unregister()

    def image_callback(self, image, info):
        self._goal = DopeGoal()
        self._goal.image = image
        self._goal.cam_info = info

    def process_data(self, data):
        # Get the current image
        self.get_next_image()
        if self._goal is None:
            return None

        # Get Dope detections
        self._client.send_goal(self._goal)
        self._client.wait_for_result(rospy.Duration(5))
        result = self._client.get_result()
        if result is None:
            self._logger.error('Failed to get result within 5s')
            return None
        self._logger.debug('Dope result is {}'.format(result))

        # Build scene graph

        return None


sense.sensors['dope_sensor'] = DopeSensor
