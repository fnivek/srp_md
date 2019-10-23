from __future__ import absolute_import
from . import sense
import logging
import rospy
import tf
import actionlib
import message_filters
from sensor_msgs.msg import CameraInfo, Image as ImageSensor_msg
from geometry_msgs.msg import PoseStamped
from dope_msgs.msg import DopeAction, DopeGoal
from srp_md_msgs.msg import DetectPlaneAction, DetectPlaneGoal
from srp_md_msgs.srv import PoseToSceneGraph, PoseToSceneGraphRequest


class DopeSensor(sense.BaseSensor):
    def __init__(self):
        super(DopeSensor, self).__init__()

        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Timing
        self._timeout = 5

        # Transforms
        self._listener = tf.TransformListener()

        # Action client
        self._dope_goal = None
        self._dope_client = actionlib.SimpleActionClient('dope', DopeAction)
        self._plane_goal = None
        self._plane_client = actionlib.SimpleActionClient('plane_detector', DetectPlaneAction)
        self._pose_to_scene_graph_client = rospy.ServiceProxy('pose_to_scene_graph', PoseToSceneGraph)

    def get_next_image(self, timeout=None):
        # Reset goal
        self._dope_goal = None

        # Start ROS subscribers
        image_sub = message_filters.Subscriber(
            '/head_camera/rgb/image_raw',
            ImageSensor_msg
        )
        info_sub = message_filters.Subscriber(
            '/head_camera/rgb/camera_info',
            CameraInfo
        )
        ts = message_filters.TimeSynchronizer([image_sub, info_sub], 10)
        ts.registerCallback(self.image_callback)

        # Wait for message with timeout
        start = rospy.get_rostime()
        if timeout is None:
            timeout = 0
        else:
            timeout = rospy.Duration(timeout)
        rate = rospy.Rate(100)
        while (self._dope_goal is None and (not rospy.is_shutdown()) and
                                           (timeout == 0 or rospy.get_rostime() - start < timeout)):
            rate.sleep()

        if self._dope_goal is None:
            self._logger.error('Failed to get an image within {}s'.format(timeout.to_sec()))

        # Unregister image subscription
        image_sub.unregister()
        info_sub.unregister()

    def image_callback(self, image, info):
        self._dope_goal = DopeGoal()
        self._dope_goal.image = image
        self._dope_goal.cam_info = info

    def process_data(self, data):
        # Start looking for the table
        self._plane_client.send_goal(self._plane_goal)

        # Get Dope detections
        #   First get the current image
        self.get_next_image(self._timeout)
        if self._dope_goal is None:
            return None
        self._dope_client.send_goal(self._dope_goal)

        # Wait for action servers to return
        self._dope_client.wait_for_result(rospy.Duration(5))
        self._plane_client.wait_for_result(rospy.Duration(5))

        # Get Dope result
        dope_result = self._dope_client.get_result()
        if dope_result is None:
            self._logger.error('Failed to get result from Dope within {}s'.format(self._timeout))
            return None
        self._logger.debug('Dope result is {}'.format(dope_result))

        # Get table result
        plane_result = self._plane_client.get_result()
        if plane_result is None:
            self._logger.error('Failed to get result from plane detector within {}s'.format(self._timeout))
            return None
        self._logger.debug('Plane detector result is {}'.format(plane_result))

        # Transform dope msgs
        for detection in dope_result.detections:
            # Make a stamped pose
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = '/head_camera_rgb_optical_frame'
            pose_stamped.header.stamp = rospy.Time(0)
            pose_stamped.pose = detection.bbox.center
            try:
                tfed_pose_stamped = self._listener.transformPose('/base_link', pose_stamped)
                detection.bbox.center = tfed_pose_stamped.pose
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                self._logger.error('Failed to transform from /head_camera_rgb_optical_frame to /base_link: {}'.format(e)
                                   )
                return None

        # Build scene graph
        req = PoseToSceneGraphRequest()
        req.names = []
        req.objects = []
        class_ids = rospy.get_param("/dope/class_ids")
        class_names = {class_id: name for name, class_id in class_ids.iteritems()}
        for detection in dope_result.detections:
            req.names.append(class_names[detection.results[0].id])
            req.objects.append(detection.bbox)
        try:
            resp = self._pose_to_scene_graph_client(req)
        except rospy.ServiceException, e:
            self._logger.error('Failed to get scene graph from pose: {}'.format(e))
            return None
        # Convert ros msg to scene graph
        self._logger.debug('Pose to scene graph result: {}'.format(resp))

        return None


sense.sensors['dope_sensor'] = DopeSensor
