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
import srp_md
from srp_md_msgs.msg import GetTableAction, GetTableGoal
from srp_md_msgs.srv import PoseToSceneGraph, PoseToSceneGraphRequest
import py_trees


class DopeSensor(sense.BaseSensor):
    def __init__(self):
        super(DopeSensor, self).__init__()

        # Initialize logger
        self._logger = logging.getLogger(__name__)

        # Set timeout
        self._timeout = 5

        # Transforms
        self._listener = tf.TransformListener()

        # Initilize properties
        # TODO(Henry): Add properties?
        self.properties = {"class": ['cracker', 'gelatin', 'meat', 'mustard', 'soup', 'sugar', 'bleach']}

        # Action clients
        self._dope_goal = None
        self._dope_client = actionlib.SimpleActionClient('dope', DopeAction)
        self._pose_to_scene_graph_client = rospy.ServiceProxy('pose_to_scene_graph', PoseToSceneGraph)

    def process_data(self, data):
        # Get Dope detections
        #   First get the current image
        self._dope_goal = DopeGoal()
        self._dope_goal.image = data["image"]
        self._dope_goal.cam_info = data["info"]
        self._dope_goal.pc = data['points']
        if self._dope_goal is None:
            return None
        self._dope_client.send_goal(self._dope_goal)

        # Wait for action servers to return
        self._dope_client.wait_for_result(rospy.Duration(self._timeout))

        # Get Dope result
        dope_result = self._dope_client.get_result()
        if dope_result is None:
            self._logger.error('Failed to get result from Dope within {}s'.format(self._timeout))
            return None
        self._logger.debug('Dope result is {}'.format(dope_result))

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
        obj_bboxes = {}
        req = PoseToSceneGraphRequest()
        req.names = []
        req.objects = []
        class_ids = rospy.get_param("/dope/class_ids")
        class_names = {class_id: name for name, class_id in class_ids.iteritems()}
        uuid = 0
        for detection in dope_result.detections:
            obj_bboxes[class_names[detection.results[0].id] + '_' + str(uuid)] = detection.bbox
            req.names.append(class_names[detection.results[0].id] + '_' + str(uuid))
            uuid += 1
            req.objects.append(detection.bbox)

        py_trees.blackboard.Blackboard().set('obj_bboxes', obj_bboxes)
        self._logger.debug('The object bboxes: {}'.format(obj_bboxes))

        try:
            resp = self._pose_to_scene_graph_client(req)
        except rospy.ServiceException, e:
            self._logger.error('Failed to get scene graph from pose: {}'.format(e))
            return None
        # Convert ros msg to scene graph
        self._logger.debug('Pose to scene graph result: {}'.format(resp))
        # Make srp_md objects
        objs = []
        for name in req.names:
            uuid = int(name[name.rfind('_') + 1:])
            label = name[:name.rfind('_')]
            objs.append(srp_md.Object(name=name, id_num=uuid, uuid=uuid, assignment={'class': label}))
        # Build the scene graph
        scene_graph = srp_md.SceneGraph(objs)
        # Update all relations from response
        for name1, name2, rel_value in zip(resp.object1, resp.object2, resp.relation):
            obj1 = srp_md.Var(uuid=int(name1[name1.rfind('_') + 1:]))
            obj2 = srp_md.Var(uuid=int(name2[name2.rfind('_') + 1:]))
            rel = scene_graph.get_rel_by_objs(obj1, obj2)
            rel.value = rel_value
            # Check if obj1 and obj2 are fliped
            if rel.obj1 != obj1:
                rel.rev_relation()

        return scene_graph


sense.sensors['dope_sensor'] = DopeSensor
