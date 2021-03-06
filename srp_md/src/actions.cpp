#include "srp_md/actions.h"

// default constructor that starts all the action clients
Act::Act(ros::NodeHandle& nh)
  : nh_(nh)
  , ac_arm_joints_("/arm_controller/follow_joint_trajectory", true)
  , ac_gripper_("/gripper_controller/gripper_action", true)
  , ac_torso_ctrl_("/torso_controller/follow_joint_trajectory", true)
  , ac_head_pan_tilt_("head_controller/follow_joint_trajectory")
  , ac_head_point_("head_controller/point_head")
  , move_group_("arm")
  , robot_model_loader_("robot_description")
  , kinematic_state_(new robot_state::RobotState(robot_model_loader_.getModel()))
  , PI(3.14159265359)
  , MAX_HEAD_PAN(PI / 2)
  , MIN_HEAD_PAN(-PI / 2)
  , MAX_HEAD_TILT(4 * PI / 9)
  , MIN_HEAD_TILT(-PI / 4)
  , MAX_TORSO_HEIGHT(0.4)
  , MIN_TORSO_HEIGHT(0.05)
  , MAX_GRIPPER_VAL(1.0)
  , MIN_GRIPPER_VAL(0.0)
  , tf2_listener_(tf2_buffer_)
{
    wait_for_server(ac_arm_joints_, "/arm_controller/follow_joint_trajectory");
    wait_for_server(ac_gripper_, "/gripper_controller/gripper_action");
    wait_for_server(ac_torso_ctrl_, "/torso_controller/follow_joint_trajectory");
    wait_for_server(ac_head_pan_tilt_, "/head_controller/follow_joint_trajectory");
    wait_for_server(ac_head_point_, "/head_controller/point_head");

    move_group_.setPlannerId("RRTConnectkConfigDefault");
    get_scene_client_ = nh_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    planning_scene_diff_pub_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    kinematic_model_ = robot_model_loader_.getModel();
    ROS_INFO("Model frame: %s", kinematic_model_->getModelFrame().c_str());
    kinematic_state_->setToDefaultValues();
    joint_model_group_ = kinematic_model_->getJointModelGroup("arm");

    // load_predefined_poses();

    // Debug please delete
    plane_cropped_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_cropped", 1000);
    plane_bounding_box_ = nh_.advertise<jsk_recognition_msgs::BoundingBox>("plane_bbox", 1000);
    object_bounding_box_ = nh_.advertise<jsk_recognition_msgs::BoundingBox>("object_bbox", 1000);

    ROS_INFO("Manipulation pipeline ready!");
}

/*  function: load_trajectory
    description:
        Loads initial trajectory for Fetch
    args:
        &plan (moveit::planning_interface::MoveGroupInterface::Plan)
        &trajectory_name (const string)
    return:
        bool (true for success)
*/
bool Act::load_trajectory(moveit::planning_interface::MoveGroupInterface::Plan& plan,
                          const std::string& trajectory_name)
{
    // If the trajectory is already pre-loaded, then set plan as that
    if (predefined_plan_map.find(trajectory_name) != predefined_plan_map.end())
    {
        plan = predefined_plan_map[trajectory_name];
        ROS_INFO("USED Preloaded traj");
        return true;
    }
    // If the trajectory is not pre-loaded, then load one!
    else
    {
        // If not such loadable trajectory exists, return false (unsuccessful)
        if (!nh_.hasParam(trajectory_name))
        {
            ROS_ERROR("Can't find pre recorded %s trajectory!", trajectory_name.c_str());
            return false;
        }

        // If such loadable trajectory exists, initialize variables
        ROS_INFO("Start loading pre recorded %s trajectory!", trajectory_name.c_str());
        bool success = true;
        int length = 0;
        float time_from_start;
        std::vector<std::string> joint_names;
        std::vector<double> positions, velocities, accelerations;

        // If getting parameters do not succeed, set return value to false
        success = success && nh_.getParam(trajectory_name + "/length", length);
        success = success && nh_.getParam(trajectory_name + "/joint_names", joint_names);

        // Input the parameters into the variable of interest
        plan.trajectory_.joint_trajectory.joint_names = joint_names;
        plan.trajectory_.joint_trajectory.points.clear();
        plan.trajectory_.joint_trajectory.points.resize(length);
        for (int i = 0; i < length; i++)
        {
            success = success && nh_.getParam(trajectory_name + "/index" + std::to_string(i) + "/positions", positions);
            success =
                success && nh_.getParam(trajectory_name + "/index" + std::to_string(i) + "/velocities", velocities);
            success = success &&
                      nh_.getParam(trajectory_name + "/index" + std::to_string(i) + "/accelerations", accelerations);
            success = success && nh_.getParam(trajectory_name + "/index" + std::to_string(i) + "/time_from_start",
                                              time_from_start);
            plan.trajectory_.joint_trajectory.points[i].positions = positions;
            plan.trajectory_.joint_trajectory.points[i].velocities = velocities;
            plan.trajectory_.joint_trajectory.points[i].accelerations = accelerations;
            plan.trajectory_.joint_trajectory.points[i].time_from_start = ros::Duration(time_from_start);
        }

        // Report on the outcome of loading trajectory
        if (!success)
            ROS_INFO("Fail to load the trajectory, the data might be broken");
        else
            ROS_INFO("Finished loading pre recorded %s trajectory!", trajectory_name.c_str());
        return success;
    }
}

/*  function: plan
    description:
        1st: Plan a trajectory to a predefined pose given its pose name
        2nd: Plan a trajectory to a pose given joint angles
        3rd: Plan a trajectory to a pose
    args:
        1st: pose_name (string)
        2nd: joint_angles (map, {joint name(str) -> joint angle(double)})
        3rd: pose (geometry_msgs::Pose)
    return:
        pair<bool (true for success), moveit::planning_interface::MoveGroupInterface::Plan>
*/
std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan> Act::plan(const std::string& pose_name,
                                                                                int max_try)
{
    // If joint angles with give pose name is not predefined, then return false with null plan
    if (predefined_joint_angles_.find(pose_name) == predefined_joint_angles_.end())
    {
        ROS_INFO("Pose %s is not predefined!", pose_name.c_str());
        return { false, moveit::planning_interface::MoveGroupInterface::Plan() };
    }

    // Ohterwise, load the joint angles and using overload, get plan with joint angles
    std::map<std::string, double> joint_angles = predefined_joint_angles_[pose_name];
    return this->plan(joint_angles, max_try);
}

std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
Act::plan(const std::map<std::string, double>& joint_angles, int max_try)
{
    ROS_INFO("PLANNING");
    if (max_try <= 0)
        max_try = 1;

    // Initialize the plan and set the target values
    move_group_.setJointValueTarget(joint_angles);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    // Until success in max number of tries, try planning
    do
    {
        success = (bool)move_group_.plan(plan);
        --max_try;
    } while (!success && max_try > 0);

    // Report if fails
    if (!success)
        ROS_INFO("Fail to plan a trajectory to the pose");

    // Return
    return std::make_pair(success, plan);
}

std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan> Act::plan(const geometry_msgs::Pose& pose,
                                                                                int max_try)
{
    ROS_INFO("PLANNING");
    if (max_try <= 0)
        max_try = 1;

    // Initialize the plan and set the target values
    move_group_.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = false;

    // Until success in max number of tries, try planning
    do
    {
        success = (bool)move_group_.plan(plan);
        --max_try;
    } while (!success && max_try > 0);

    // Report if fails
    if (!success)
        ROS_INFO("Fail to plan a trajectory given the geometry pose");

    // Return
    return std::make_pair(success, plan);
}

/*  function: boolean_interface
    description:
        Prompt a yes or no question to the user
    args:
        action (string)
    return:
        bool (true for yes)
*/
bool Act::boolean_interface(const std::string& action)
{
    // Prompt the question
    char c;
    ROS_INFO("Do you want to %s? Enter y(yes) or n(no)", action.c_str());

    // Take in the input
    std::cin >> c;

    // If input fails or with wrong input,
    while (std::cin.fail() || (c != 'y' && c != 'n'))
    {
        // Clear and redo the input
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        ROS_INFO("Wrong Command. Please enter y(yes) or n(no)");
        std::cin >> c;
    }

    // Return true if 'y' and false if 'c'
    return c == 'y';
}

/*  function: record_trajectory
    description:
        Save a trajectory to a yaml file
    args:
        &plan (moveit::planning_interface::MoveGroupInterface::Plan)
        name (string)
    return:
        bool (true for success)
*/
bool Act::record_trajectory(const moveit::planning_interface::MoveGroupInterface::Plan& plan, std::string name)
{
    ROS_INFO("***********Begin to write into a %s.yaml file...***********", name.c_str());

    // Specify where to output file
    // std::string filename = "/home/logan/" + name + ".yaml";
    std::string filename = "/home/jihwangk" + name + ".yaml";

    // Try opening the file, return false if fails
    std::ofstream outfile(filename);
    if (outfile.fail())
    {
        ROS_INFO("Cannot open %s, the file might be in use", filename.c_str());
        return false;
    }

    // Write necessary info to the yaml file
    outfile << name + ":" << std::endl;
    outfile << "  length: " << plan.trajectory_.joint_trajectory.points.size() << std::endl;
    outfile << "  joint_names: [";
    for (int j = 0; j < plan.trajectory_.joint_trajectory.joint_names.size(); j++)
    {
        if (j == plan.trajectory_.joint_trajectory.joint_names.size() - 1)
            outfile << "'" + plan.trajectory_.joint_trajectory.joint_names[j] + "']" << std::endl;
        else
            outfile << "'" + plan.trajectory_.joint_trajectory.joint_names[j] + "', ";
    }
    for (int i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        outfile << "  index" << i << ":" << std::endl;
        outfile << "    positions: [";
        for (int j = 0; j < plan.trajectory_.joint_trajectory.points[i].positions.size(); j++)
        {
            if (j == plan.trajectory_.joint_trajectory.points[i].positions.size() - 1)
                outfile << plan.trajectory_.joint_trajectory.points[i].positions[j] << "]" << std::endl;
            else
                outfile << plan.trajectory_.joint_trajectory.points[i].positions[j] << ", ";
        }
        outfile << "    velocities: [";
        for (int j = 0; j < plan.trajectory_.joint_trajectory.points[i].velocities.size(); j++)
        {
            if (j == plan.trajectory_.joint_trajectory.points[i].velocities.size() - 1)
                outfile << plan.trajectory_.joint_trajectory.points[i].velocities[j] << "]" << std::endl;
            else
                outfile << plan.trajectory_.joint_trajectory.points[i].velocities[j] << ", ";
        }
        outfile << "    accelerations: [";
        for (int j = 0; j < plan.trajectory_.joint_trajectory.points[i].accelerations.size(); j++)
        {
            if (j == plan.trajectory_.joint_trajectory.points[i].accelerations.size() - 1)
                outfile << plan.trajectory_.joint_trajectory.points[i].accelerations[j] << "]" << std::endl;
            else
                outfile << plan.trajectory_.joint_trajectory.points[i].accelerations[j] << ", ";
        }
        outfile << "    time_from_start: " << plan.trajectory_.joint_trajectory.points[i].time_from_start << std::endl;
    }

    // Close the file and report & return
    outfile.close();
    ROS_INFO("***********Finished to write into a %s.yaml file***********", name.c_str());
    return true;
}

void Act::crop_box_filt_pcl_pc(const pcl::PCLPointCloud2::Ptr pcl_in_pc, const vision_msgs::BoundingBox3D& crop_box,
                               pcl::PCLPointCloud2& pcl_out_pc, bool invert)
{
    pcl::CropBox<pcl::PCLPointCloud2> filt;
    filt.setInputCloud(pcl_in_pc);
    // Generate the max and min points of the bounding box
    Eigen::Affine3f tf = Eigen::Translation<float, 3>(crop_box.center.position.x, crop_box.center.position.y,
                                                      crop_box.center.position.z) *
                         Eigen::Quaternionf(crop_box.center.orientation.w, crop_box.center.orientation.x,
                                            crop_box.center.orientation.y, crop_box.center.orientation.z);
    Eigen::Vector4f max_pt(crop_box.size.x / 2, crop_box.size.y / 2, crop_box.size.z / 2, 1);
    Eigen::Vector4f min_pt(-crop_box.size.x / 2, -crop_box.size.y / 2, -crop_box.size.z / 2, 1);
    filt.setMax(max_pt);
    filt.setMin(min_pt);
    filt.setTransform(tf.inverse());
    filt.setNegative(invert);
    filt.filter(pcl_out_pc);
}

// Pont clouds
void Act::crop_box_filt_pc(const sensor_msgs::PointCloud2::Ptr in_pc, const vision_msgs::BoundingBox3D& crop_box,
                           sensor_msgs::PointCloud2& out_pc, bool invert)
{
    // Convert to pcl point cloud 2
    pcl::PCLPointCloud2::Ptr pcl_in_pc(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 pcl_out_pc;
    pcl_conversions::toPCL(*in_pc, *pcl_in_pc);
    // Make a crop box filter
    crop_box_filt_pcl_pc(pcl_in_pc, crop_box, pcl_out_pc, invert);

    // Move output data over
    pcl_conversions::moveFromPCL(pcl_out_pc, out_pc);
}

void Act::transform_pc(const sensor_msgs::PointCloud2& in_pc, std::string frame_id, sensor_msgs::PointCloud2& out_pc)
{
    tf_listener_.waitForTransform("/base_link", in_pc.header.frame_id, in_pc.header.stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud(frame_id, in_pc, out_pc, tf_listener_);
}

// Functions below needs to be changed!
/*  function: cartesian_grasp
    description:
        grasp object through cartesian move
    args:
        waypoints (vector<geometry_msgs::Pose>)
        max_try (int, default is 3)
    return:
        bool (true for success)
*/
bool Act::cartesian_grasp(const std::vector<geometry_msgs::Pose>& waypoints, int max_try /* = 3 */)
{
    open_gripper();
    auto cartesian_move_result = cartesian_move(waypoints, max_try);
    close_gripper();  // not sure if close_gripper() will return true or false if an object is grasped
    return cartesian_move_result.first;
}

/*  function: cartesian_move(overload version(1/2))
    description:
        cartesian move the arm given the end pose
    args:
        end_pose (geometry_msgs::Pose)
    return:
        pair<bool (true for success), moveit::planning_interface::MoveGroupInterface::Plan>
*/

std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
Act::relative_cartesian_move(const geometry_msgs::TransformStamped& pose_diff_msg, int max_try /* = 3 */)
{
    Eigen::Affine3d pose_diff = tf2::transformToEigen(pose_diff_msg);
    if (pose_diff_msg.header.frame_id != "base_link")
    {
        // Transform the transform to "base_link"
        geometry_msgs::TransformStamped tf;
        try
        {
            // Get transform to base_link
            tf = tf2_buffer_.lookupTransform("base_link", pose_diff_msg.header.frame_id, ros::Time(0));
            // Convert to Eigen
            Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf);
            // Only apply rotation to translation
            pose_diff.translation() = tf_eigen.linear() * pose_diff.translation();
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            return { false, plan };
        }
    }

    // Get the current pose
    geometry_msgs::Pose current_pose_msg = move_group_.getCurrentPose().pose;
    Eigen::Affine3d current_pose;
    tf2::fromMsg(current_pose_msg, current_pose);

    // Apply relative transform to current_pose
    pose_diff.linear() *= current_pose.linear();
    pose_diff.translation() += current_pose.translation();

    geometry_msgs::Pose computed = tf2::toMsg(pose_diff);

    std::vector<geometry_msgs::Pose> waypoints = { computed };
    return cartesian_move(waypoints, max_try);
}

std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
Act::cartesian_move(const geometry_msgs::TransformStamped& pose_diff_msg, int max_try /* = 3 */)
{
    Eigen::Affine3d pose_diff = tf2::transformToEigen(pose_diff_msg);
    if (pose_diff_msg.header.frame_id != "base_link")
    {
        // Transform the transform to "base_link"
        geometry_msgs::TransformStamped tf;
        try
        {
            // Get transform to base_link
            tf = tf2_buffer_.lookupTransform("base_link", pose_diff_msg.header.frame_id, ros::Time(0));
            // Convert to Eigen
            Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf);
            // Only apply rotation to translation
            pose_diff = tf_eigen * pose_diff;
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            return { false, plan };
        }
    }

    // Get the current pose
    geometry_msgs::Pose current_pose_msg = move_group_.getCurrentPose().pose;
    Eigen::Affine3d current_pose;
    tf2::fromMsg(current_pose_msg, current_pose);

    // Apply relative transform to current_pose
    // pose_diff.linear() *= current_pose.linear();
    // pose_diff.translation() += current_pose.translation();

    geometry_msgs::Pose computed = tf2::toMsg(pose_diff);

    std::vector<geometry_msgs::Pose> waypoints = { computed };
    return cartesian_move(waypoints, max_try);
}

/*  function: cartesian_move(overload version(2/2))
    description:
        cartesian move the arm given the a vector of waypoints
    args:
        end_pose (vector<geometry_msgs::Pose>)
    return:
        pair<bool (true for success), moveit::planning_interface::MoveGroupInterface::Plan>
*/
std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
Act::cartesian_move(const std::vector<geometry_msgs::Pose>& waypoints, int max_try /* = 3 */)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (waypoints.empty())
        return { false, plan };
    if (max_try <= 0)
        max_try = 1;

    std::pair<bool, moveit_msgs::RobotTrajectory> plan_result = cartesian_plan(waypoints, max_try);
    if (!plan_result.first)
        return { false, plan };

    moveit_msgs::RobotTrajectory& trajectory_msg = plan_result.second;

    // The trajectory needs to be modified so it will include velocities as well.
    // First to create a RobotTrajectory object
    robot_trajectory::RobotTrajectory rt(move_group_.getCurrentState()->getRobotModel(), "arm");

    // Second get a RobotTrajectory from trajectory
    rt.setRobotTrajectoryMsg(*move_group_.getCurrentState(), trajectory_msg);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;

    // Fourth compute computeTimeStamps
    bool success = false;
    do
    {
        success = iptp.computeTimeStamps(rt);
        ROS_INFO("Computed time stamp %s", success ? "SUCCEDED" : "FAILED");
        --max_try;
        if (!success && max_try > 0)
            ROS_INFO("Try again");
    } while (!success && max_try > 0);
    if (!success)
        return { false, plan };

    // Get RobotTrajectory_msg from RobotTrajectory
    rt.getRobotTrajectoryMsg(trajectory_msg);

    // Finally plan and execute the trajectory
    ROS_INFO("Start execute the plan!");
    plan.trajectory_ = trajectory_msg;
    success = (bool)move_group_.execute(plan);
    if (!success)
        ROS_INFO("Cartesian move fail");
    return { success, plan };
}

/*  function: cartesian_plan(overload version (1/2))
    description:
        plan a cartesian trajectory given its end pose
    args:
        end_pose (geometry_msgs::Pose)
    return:
        pair<bool (true for success), moveit_msgs::RobotTrajectory>
*/
std::pair<bool, moveit_msgs::RobotTrajectory> Act::cartesian_plan(const geometry_msgs::Pose& end_pose,
                                                                  int max_try /* = 1 */)
{
    std::vector<geometry_msgs::Pose> waypoints = { end_pose };
    return cartesian_plan(waypoints, max_try);
}

/*  function: cartesian_plan(overload version(2/2))
    description:
        plan a cartesian trajectory given a vector of waypoints (current pose is excluded)
    args:
        waypoints (vector<geometry_msgs::Pose>)
    return:
        pair<bool (true for success), moveit_msgs::RobotTrajectory>
*/
std::pair<bool, moveit_msgs::RobotTrajectory> Act::cartesian_plan(const std::vector<geometry_msgs::Pose>& waypoints,
                                                                  int max_try /* = 1 */)
{
    if (max_try <= 0)
        max_try = 1;
    moveit_msgs::RobotTrajectory trajectory_msg;

    double eef_step = 0.1, jump_threshold = 0.0;
    bool avoid_collision = false;
    move_group_.setPlanningTime(10.0);
    double fraction = 0.0;
    do
    {
        fraction =
            move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory_msg, avoid_collision);
        ROS_INFO("Cartesian planner achieved (%.2f%%)", fraction * 100);
        --max_try;
        if (fraction != 1.0 && max_try > 0)
            ROS_INFO("Try again");
    } while (fraction != 1.0 && max_try > 0);

    return { fraction == 1.0, trajectory_msg };
}

/*  function: control_gripper
    description:
        open or close gripper to some extent
    args:
        pos (double, from 0.0 ~ 1.0)
    return:
        bool (true for success)
*/
bool Act::control_gripper(double pos)
{
    control_msgs::GripperCommandGoal goal;
    goal.command.position = pos;
    goal.command.max_effort = 0.0;
    ac_gripper_.sendGoal(goal);
    bool success = ac_gripper_.waitForResult(ros::Duration(3.0));
    if (success)
        ROS_INFO("Successly controlled the gripper, gripper pos:%f", pos);
    else
        ROS_INFO("Fail to control the gripper");
    return success;
}

/*  function: open_gripper
    description:
        open gripper(to maximum extent)
    args:
        void
    return:
        bool (true for success)
*/
bool Act::open_gripper()
{
    return control_gripper(MAX_GRIPPER_VAL);
}

/*  function: close_gripper
    description:
        close gripper(to mimimum extent)
    args:
        void
    return:
        bool (true for success)
*/
bool Act::close_gripper()
{
    return control_gripper(MIN_GRIPPER_VAL);
}

/*  function: move(overload version(1/3))
    description:
        move arm to one of the predefined poses given its pose name
    args:
        pose_name (string)
    return:
        pair<bool (true for success), moveit::planning_interface::MoveGroupInterface::Plan>
*/
bool Act::move(const std::string& pose_name, int max_try /* = 1 */)
{
    move_group_.setStartStateToCurrentState();
    auto plan_result = this->plan(pose_name, max_try);
    if (!plan_result.first)
        return false;
    bool success = (bool)move_group_.execute(plan_result.second);
    if (!success)
        ROS_INFO("Fail to move to %s", pose_name.c_str());
    return success;
}

/*  function: move(overload v(2/3))
    description:
        move arm to some pose given its joint angles
    args:
        joint_angles ( map {joint name -> joint angle} )
    return:
        bool (true for success)
*/
bool Act::move(const std::map<std::string, double>& joint_angles, int max_try /* = 1 */)
{
    move_group_.setStartStateToCurrentState();
    auto plan_result = this->plan(joint_angles, max_try);
    if (!plan_result.first)
        return false;
    bool success = (bool)move_group_.execute(plan_result.second);
    if (!success)
        ROS_INFO("Fail to move given the joint angles");
    return success;
}

/*  function: move(overload v(3/3))
    description:
        move arm to some pose given its geometry pose
    args:
        pose (geometry_msgs::Pose)
    return:
        bool (true for success)
*/
bool Act::move(const geometry_msgs::Pose& pose, int max_try /* = 1 */)
{
    move_group_.setStartStateToCurrentState();
    auto plan_result = this->plan(pose, max_try);
    if (!plan_result.first)
        return false;
    bool success = (bool)move_group_.execute(plan_result.second);
    if (!success)
        ROS_INFO("Fail to move given the geometry pose");
    return success;
}

/*  function: move(overload v(4/3))
    description:
        move arm to some pose given plan
    args:
        plan (moveit::planning_interface::MoveGroupInterface::Plan )
    return:
        bool (true for success)
*/
bool Act::move(moveit::planning_interface::MoveGroupInterface::Plan move_plan)
{
    bool success = (bool)move_group_.execute(move_plan);
    if (!success)
        ROS_INFO("Fail to move to the given plan");
    return success;
}

/*  function: relative_move()
    description:
        move arm to some pose relative to current pose
    args:
        pose_diff (geometry_msgs::TransformStamped)
    return:
        bool (true for success)
*/
bool Act::relative_move(const geometry_msgs::TransformStamped& pose_diff_msg, int max_try /* = 1 */)
{
    // Convert to Eigen
    Eigen::Affine3d pose_diff = tf2::transformToEigen(pose_diff_msg);
    if (pose_diff_msg.header.frame_id != "base_link")
    {
        // Transform the transform to "base_link"
        geometry_msgs::TransformStamped tf;
        try
        {
            // Get transform to base_link
            tf = tf2_buffer_.lookupTransform("base_link", pose_diff_msg.header.frame_id, ros::Time(0));
            // Convert to Eigen
            Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf);
            // Only apply rotation to translation
            pose_diff.translation() = tf_eigen.linear() * pose_diff.translation();
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            return false;
        }
    }

    // Get the current pose
    geometry_msgs::Pose current_pose_msg = move_group_.getCurrentPose().pose;
    Eigen::Affine3d current_pose;
    tf2::fromMsg(current_pose_msg, current_pose);

    // Apply relative transform to current_pose
    pose_diff.linear() *= current_pose.linear();
    pose_diff.translation() += current_pose.translation();

    geometry_msgs::Pose computed = tf2::toMsg(pose_diff);
    auto plan_result = this->plan(computed, max_try);
    if (!plan_result.first)
        return false;
    bool success = (bool)move_group_.execute(plan_result.second);
    if (!success)
        ROS_INFO("Fail to move given relative pose");
    return success;
}
/*
 * Find a pose to stack the object on top of another
 * Inputs:
 *      bot_pose - pose of bottom object
 *      bot_dim  - dimension of bottom object
 *      dim      - dimension of upper object
 * Output:
 *      upper_pose_msg - pose of upper object
 */
geometry_msgs::Pose Act::GetStackPose(geometry_msgs::Pose bot_pose, geometry_msgs::Vector3 bot_dim,
                                      geometry_msgs::Vector3 dim)
{
    // convert input pose from geometry_msgs to tf2
    tf2::Transform bot_pose_tf;
    tf2::fromMsg(bot_pose, bot_pose_tf);

    // convert input dimension from geometry_msgs to tf2
    tf2::Vector3 bot_dim_tf(0, 0, 0);
    tf2::Vector3 dim_tf(0, 0, 0);
    tf2::convert(bot_dim, bot_dim_tf);
    tf2::convert(dim, dim_tf);

    // create coefficient the dimension of object to multiply by to get points coordinate in its own frame
    float length_coeff[8] = { 0.5, 0.5, 0.5, 0.5, -0.5, -0.5, -0.5, -0.5 };
    float width_coeff[8] = { 0.5, 0.5, -0.5, -0.5, 0.5, 0.5, -0.5, -0.5 };
    float height_coeff[8] = { 0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5 };

    // initialize vector to store points
    std::vector<tf2::Vector3> bot_points_botframe;
    std::vector<tf2::Vector3> bot_points_world;

    // loop through all points to find highest point
    float max_z_value = -1;
    int max_index = -1;
    std::cout << sizeof(length_coeff) / sizeof(*length_coeff) << std::endl;
    for (int i = 0; i < sizeof(length_coeff) / sizeof(*length_coeff); i++)
    {
        bot_points_botframe.push_back(tf2::Vector3(bot_dim_tf.x() * length_coeff[i], bot_dim_tf.y() * width_coeff[i],
                                                   bot_dim_tf.z() * height_coeff[i]));

        bot_points_world.push_back(bot_pose_tf * bot_points_botframe[i]);
        if (bot_points_world[i].z() > max_z_value)
        {
            max_z_value = bot_points_world[i].z();
            max_index = i;
        }
    }
    // get the radius and pose of upper object
    tf2::Vector3 origin_bot = bot_pose_tf.getOrigin();
    float z_radius = sqrt(dim_tf.x() * dim_tf.x() + dim_tf.y() * dim_tf.y() + dim_tf.z() * dim_tf.z()) / 2;
    // keep same orientation as bottom one and add half of radius to z axis
    tf2::Transform pose_upper_tf(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(origin_bot.x(), origin_bot.y(),
                                                                           bot_points_world[max_index].z() + z_radius));

    // convert output to geometry_msgs form
    geometry_msgs::Pose upper_pose_msg;
    tf2::toMsg(pose_upper_tf, upper_pose_msg);
    return upper_pose_msg;
}

/*  function: get_table()
    description:
        detects the table
    args:
        points (sensor_msgs::PointCloud2)
    return:

*/
bool Act::get_table(const sensor_msgs::PointCloud2::ConstPtr& points,
                    std::vector<vision_msgs::BoundingBox3D>& plane_bboxes)
{
    // Initialize variables
    sensor_msgs::PointCloud2::Ptr points_tf(new sensor_msgs::PointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>),
        cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    float tolerance_ang = 15.0 * PI / 180;

    // Transform the pointcloud to base frame
    transform_pc(*points, "base_link", *points_tf);

    // Convert to PCL PointCloudc
    pcl::PCLPointCloud2::Ptr points_pcl2(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_pcl(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*points_tf, *points_pcl2);
    pcl::fromPCLPointCloud2(*points_pcl2, *points_pcl);

    // Do parametric segmentation
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);

    // Creating filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Save the original size of point cloud
    int i = 0, nr_points = (int)points_pcl->points.size();

    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_cloud_vector;
    std::vector<tf2::Vector3> sizes_tf_vector;
    std::vector<tf2::Transform> poses_tf_vector;
    while (points_pcl->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(points_pcl);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            break;
        }
        // plane_cloud_vector.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
        // Extract the inliers
        extract.setInputCloud(points_pcl);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        sensor_msgs::PointCloud2 points_cloud_msg;
        pcl::toROSMsg(*cloud_p, points_cloud_msg);
        // test_plane_pub_.publish(points_cloud_msg);

        // std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data
        // points, out of " << nr_points << " total points." << std::endl; std::stringstream ss;
        // ss << "table_scene_lms400_plane_" << i << ".pcd";
        // writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        points_pcl.swap(cloud_f);
        i++;

        // Normalize the coefficients A, B and C from [A, B, C, D] and take dot product with [0, 0, 1] (essentially just
        // C normalized) If the value is greater than cos(5deg), get the table by getting min_x, min_y, max_x, max_y for
        // bounding box and averaging them for centroid. Width = (max_x - min_x), Height = (max_y - min_y). Make pose
        // with zero quaternion and (avg_x, avg_y, avg_z) and append to poses vector. Make Vector3 with (width, height,
        // degree) and save to sizes vector
        float norm_div =
            sqrt(coefficients->values[0] * coefficients->values[0] + coefficients->values[1] * coefficients->values[1] +
                 coefficients->values[2] * coefficients->values[2]);
        float dotproduct = coefficients->values[2] / norm_div;
        if (dotproduct >= cos(tolerance_ang))
        {
            pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
            feature_extractor.setInputCloud(cloud_p);
            feature_extractor.compute();
            pcl::PointXYZ min_point_AABB;
            pcl::PointXYZ max_point_AABB;
            pcl::PointXYZ position_AABB;
            // Eigen::Matrix3f rotational_matrix_AABB;
            feature_extractor.getAABB(min_point_AABB, max_point_AABB);
            sizes_tf_vector.push_back(tf2::Vector3(max_point_AABB.x - min_point_AABB.x,
                                                   max_point_AABB.y - min_point_AABB.y, acos(dotproduct)));
            float quat_x, quat_y, quat_z, quat_w;
            quat_x = coefficients->values[1];
            quat_y = coefficients->values[0];
            quat_z = 0.0;
            quat_w = sqrt(1.0 + sqrt(pow(coefficients->values[0], 2) + pow(coefficients->values[1], 2) +
                                     pow(coefficients->values[2], 2))) +
                     coefficients->values[2];
            poses_tf_vector.push_back(tf2::Transform(tf2::Quaternion(quat_x, quat_y, quat_z, quat_w),
                                                     tf2::Vector3((min_point_AABB.x + max_point_AABB.x) / 2,
                                                                  (min_point_AABB.y + max_point_AABB.y) / 2,
                                                                  (min_point_AABB.z + max_point_AABB.z) / 2)));
        }
        else
        {
            // plane_cloud_vector.pop_back();
        }
    }
    // rviz
    std::vector<float> plane_area_copy;
    std::vector<float> plane_area;
    int plane_index;
    for (auto&& size : sizes_tf_vector)
    {
        plane_area.push_back(size.x() * size.y());
        plane_area_copy.push_back(size.x() * size.y());
    }

    for (int i = 1; i < plane_area.size(); i++)
    {
        float key = plane_area[i];
        int j = i - 1;
        while ((j >= 0) && (key > plane_area[j]))
        {
            plane_area[j + 1] = plane_area[j];
            j--;
        }
        plane_area[j + 1] = key;
    }
    std::vector<float>::iterator iter;
    for (int i = 0; i < plane_area.size(); i++)
    {
        iter = find(plane_area_copy.begin(), plane_area_copy.end(), plane_area[i]);
        plane_index = std::distance(plane_area_copy.begin(), iter);
        // std::cerr<<"area: "<<"index: "<<plane_index<<" "<<plane_area[i]<<std::endl;
        geometry_msgs::Vector3 size_msg;
        geometry_msgs::Pose pose_msg;
        tf2::convert(sizes_tf_vector[plane_index], size_msg);
        tf2::toMsg(poses_tf_vector[plane_index], pose_msg);
        // std::cerr<<"size_msg: "<<"index: "<<plane_index<<std::endl<<size_msg<<std::endl;
        // std::cerr<<"pose_msg: "<<"index: "<<plane_index<<std::endl<<pose_msg<<std::endl;
        vision_msgs::BoundingBox3D plane_bbox;
        plane_bbox.center = pose_msg;
        plane_bbox.size = size_msg;
        if (plane_bbox.center.position.z > 0.5)
        {
            plane_bboxes.push_back(plane_bbox);
        }
    }
    // std::cout<<"all planes: "<<plane_bboxes.size()<<std::endl;
    bool success = true;
    return success;
}

void Act::attach_object_to_gripper(const std::string& object_name)
{
    std::vector<std::string> touch_links = { "r_gripper_finger_link", "l_gripper_finger_link", "gripper_link" };
    move_group_.attachObject(object_name, "gripper_link", touch_links);
    ROS_INFO("Attach %s", object_name.c_str());
}

/*  function: detached_object
    description:
        detach an object from given the object name
    args:
        object_name (string)
    return:
        void
*/
void Act::detach_object(const std::string& object_name)
{
    move_group_.detachObject(object_name);
    ROS_INFO("Detach %s", object_name.c_str());
}

/*  function: free_space_finder()
    description:
        finds a free space of the table
    args:

    return:

*/

/*
    string relation: None, Right, Left, Front, Back
*/

bool Act::free_space_finder(const sensor_msgs::PointCloud2::ConstPtr& points,
                            const vision_msgs::BoundingBox3D& plane_bbox, const vision_msgs::BoundingBox3D& obj_bbox,
                            const std::string& relation, const vision_msgs::BoundingBox3D& relative_obj_bbox,
                            std::vector<geometry_msgs::Pose>& pose, const float distance)
// const geometry_msgs::Vector3& obj_dim, geometry_msgs::Pose& pose)
{
    double x_axis_offset = 0.04;
    // std::cerr<<"pose of the table: "<<plane_bbox.center<<std::endl;
    std::cerr << "size of the table: " << plane_bbox.size << std::endl;
    // std::cout<<"pose.size() before function: "<<pose.size()<<std::endl;
    pose.clear();
    // enum x_axix = {Right = -1, none = 0, Left = 1};
    // enum y_axix = {Front = -1, none = 0, Back = 1}; //need to be fixed
    // x_axix x_axix_offset = none;
    // y_axix y_axix_offset = none;
    // auto it_x = relation.find(“Right”);
    // if (it_x != std::string::npos) {
    //     x_axix = Right;
    // }
    // it_x = relation.find(“Left”);
    // if (it_x != std::string::npos) {
    //     x_axix = Left;
    // }
    // auto it_y = relation.find(“Front”);
    // if (it_y != std::string::npos) {
    //     y_axix = Front;
    // }
    // it_y = relation.find(“Back”);
    // if (it_y != std::string::npos) {
    //     y_axix = Back;
    // }
    // Initialize variables
    bool success = false;
    sensor_msgs::PointCloud2::Ptr points_tf(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr points_plane_filtered(new sensor_msgs::PointCloud2);
    sensor_msgs::PointCloud2::Ptr points_object_filtered(new sensor_msgs::PointCloud2);
    vision_msgs::BoundingBox3D plane_bbox_mod = plane_bbox;
    float edge_deduction_ratio = 0.17;
    float plane_expansion_ratio = 3.0;
    float object_placing_offset = 0.03;
    int points_threshold = 0;

    // Transform the pointcloud to base frame
    transform_pc(*points, "base_link", *points_tf);
    cout << "points.header.frame_id"<< points->header.frame_id << endl << endl;
    // Crop out the plane
    plane_bbox_mod.size.z = (1 + plane_expansion_ratio) * plane_bbox.size.z;
    cout << "plane_bbox.size.z: " << plane_bbox.size.z << endl;
    cout << "plane_bbox_mod: " <<plane_bbox_mod<<endl;
    // crop_box_filt_pc(points_tf, plane_bbox_mod, *points_plane_filtered, true);
    crop_box_filt_pc(points_tf, plane_bbox_mod, *points_plane_filtered, true);
    // plane_cropped_.publish(points_plane_filtered);
    plane_cropped_.publish(points_plane_filtered);
    jsk_recognition_msgs::BoundingBox plane_bbox_jsk;
    plane_bbox_jsk.pose = plane_bbox_mod.center;
    plane_bbox_jsk.dimensions = plane_bbox_mod.size;
    plane_bbox_jsk.header.frame_id = "base_link";
    plane_bounding_box_.publish(plane_bbox_jsk);
    // std::cout<<"plane_bbox_jsk: "<<plane_bbox_jsk<<std::endl;

    // Initialize structures for random sampling
    static std::default_random_engine e(time(0));
    // static std::normal_distribution<double> n(MU,SIGMA);
    static std::uniform_real_distribution<double> dis(0.0 + edge_deduction_ratio / 2, 1.0 - edge_deduction_ratio / 2);
    // std::cout<<"teyseyshuehgiughytghjk"<<std::endl;
    // std::cout<<distance<<std::endl;
    // std::cout<<relative_obj_bbox.center.orientation.x<<std::endl;
    // std::cout<<relative_obj_bbox.center.orientation.y<<std::endl;
    // std::cout<<relative_obj_bbox.center.orientation.z<<std::endl;
    // std::cout<<relative_obj_bbox.center.orientation.w<<std::endl;
    // std::cout<<obj_bbox.size<<std::endl;
    Eigen::Quaterniond obj_quat = Eigen::Quaterniond(obj_bbox.center.orientation.w, obj_bbox.center.orientation.x,
                                                     obj_bbox.center.orientation.y, obj_bbox.center.orientation.z);
    Eigen::Vector3d z_axix = Eigen::Vector3d(0, 0, 1);
    // std::cout<<"obj_quat * z_axix: "<<obj_quat * z_axix<<std::endl;
    // std::cout<<"obj_quat.inverse()  * z_axix: "<<obj_quat.inverse() * z_axix<<std::endl;
    Eigen::Vector3d z_axix_rotated = obj_quat.inverse() * z_axix;
    enum axis_prolonged
    {
        yes = 1,
        no = 0
    };
    axis_prolonged x_axis = no;
    axis_prolonged y_axis = no;
    axis_prolonged z_axis = no;
    if (abs(z_axix_rotated(0, 0)) > abs(z_axix_rotated(1, 0)) && abs(z_axix_rotated(0, 0)) > abs(z_axix_rotated(2, 0)))
    {
        x_axis = yes;
        cout << "x_axis: " << endl;
    }
    else if (abs(z_axix_rotated(1, 0)) > abs(z_axix_rotated(0, 0)) &&
             abs(z_axix_rotated(1, 0)) > abs(z_axix_rotated(2, 0)))
    {
        y_axis = yes;
        cout << "y_axis: " << endl;
    }
    else
    {
        z_axis = yes;
        cout << "z_axis: " << endl;
    }
    // std::cout<<"abs(z_axix_rotated(0,0)): "<<abs(z_axix_rotated(0,0))<<std::endl;
    // std::cout<<"abs(z_axix_rotated(1,0)): "<<abs(z_axix_rotated(1,0))<<std::endl;
    // std::cout<<"abs(z_axix_rotated(2,0)): "<<abs(z_axix_rotated(2,0))<<std::endl;
    // std::cout<<"x_axis: "<<x_axis<<std::endl;
    // std::cout<<"y_axis: "<<y_axis<<std::endl;
    // std::cout<<"z_axis: "<<z_axis<<std::endl;
    std::map<double, geometry_msgs::Pose> dis_pose_map;
    // cout<<"testing2"<<endl<<endl;
    // cout<<"obj_bbox.center.orientation: "<<obj_bbox.center.orientation<<endl;
    // Might want to cap this at limit 1000 or sth
    for (int loop_times = 0; loop_times < 3000; loop_times++)
    {
        // Generate random pose on the table
        double x_random, y_random;
        x_random = dis(e);
        y_random = dis(e);
        // std::cout<<"Coordinates on plane: "<<x_random<<", "<<y_random<<std::endl;
        geometry_msgs::Pose test_object_pose;
        test_object_pose.position.x =
            plane_bbox.center.position.x + x_random * plane_bbox.size.x - plane_bbox.size.x / 2;
        test_object_pose.position.y =
            plane_bbox.center.position.y + y_random * plane_bbox.size.y - plane_bbox.size.y / 2;
        test_object_pose.position.z = plane_bbox.center.position.z + obj_bbox.size.z * 0.5 + object_placing_offset;
        if (y_random > 0.85)
        {
            // cout<<"test_object_pose.position.y: "<<test_object_pose.position.y<<endl;
        }
        test_object_pose.orientation = obj_bbox.center.orientation;
        double distance_actual = 1000;
        // cout<<"testing3"<<endl<<endl;
        // std::cout<<1<<std::endl;
        if (distance != 0)
        {
            // std::cout<<distance_actual<<std::endl;
            distance_actual = sqrt((relative_obj_bbox.center.position.x - test_object_pose.position.x) *
                                       (relative_obj_bbox.center.position.x - test_object_pose.position.x) +
                                   (relative_obj_bbox.center.position.y - test_object_pose.position.y) *
                                       (relative_obj_bbox.center.position.y - test_object_pose.position.y) +
                                   (relative_obj_bbox.center.position.z - test_object_pose.position.z) *
                                       (relative_obj_bbox.center.position.z - test_object_pose.position.z));
            // cout<<"before distance_actual: "<<distance_actual<<endl;
            distance_actual = distance_actual -
                              sqrt(relative_obj_bbox.size.x * relative_obj_bbox.size.x +
                                   relative_obj_bbox.size.y * relative_obj_bbox.size.y +
                                   relative_obj_bbox.size.z * relative_obj_bbox.size.z) /
                                  2 -
                              sqrt(obj_bbox.size.x * obj_bbox.size.x + obj_bbox.size.y * obj_bbox.size.y +
                                   obj_bbox.size.z * obj_bbox.size.z) /
                                  2;
            // cout<<"sqrt(relative_obj_bbox.size.x * relative_obj_bbox.size.x + relative_obj_bbox.size.y *
            // relative_obj_bbox.size.y + relative_obj_bbox.size.z * relative_obj_bbox.size.z):
            // "<<sqrt(relative_obj_bbox.size.x * relative_obj_bbox.size.x + relative_obj_bbox.size.y *
            // relative_obj_bbox.size.y + relative_obj_bbox.size.z * relative_obj_bbox.size.z)<<endl;
            // cout<<"sqrt(obj_bbox.size.x * obj_bbox.size.x + obj_bbox.size.y * obj_bbox.size.y + obj_bbox.size.z *
            // obj_bbox.size.z): "<<sqrt(obj_bbox.size.x * obj_bbox.size.x + obj_bbox.size.y * obj_bbox.size.y +
            // obj_bbox.size.z * obj_bbox.size.z)<<endl; cout<<"after distance_actual: "<<distance_actual<<endl;
            if (distance_actual > distance)
            {
                continue;
            }
        }
        // cout<<"testing3"<<endl<<endl;
        vision_msgs::BoundingBox3D test_bbox;
        test_bbox.center = test_object_pose;

        // cout<<"test_bbox.center.position.x before: "<<test_bbox.center.position.x<<endl;
        // test_bbox.center.position.x = test_bbox.center.position.x + x_axis_offset;
        // cout<<"test_bbox.center.position.x after: "<<test_bbox.center.position.x<<endl;
        // test_bbox.size = obj_dim;
        test_bbox.size.x = obj_bbox.size.x + x_axis * obj_bbox.size.x * 3;
        test_bbox.size.y = obj_bbox.size.y + y_axis * obj_bbox.size.y * 3;
        test_bbox.size.z = obj_bbox.size.z + z_axis * obj_bbox.size.z * 3;
        crop_box_filt_pc(points_plane_filtered, test_bbox, *points_object_filtered, false);
        int num_points = points_object_filtered->height * points_object_filtered->width;
        // test_bbox.center.position.x = test_bbox.center.position.x - x_axis_offset;
        // std::cout<<"point num in cropbox: "<<num_points<<std::endl;
        jsk_recognition_msgs::BoundingBox object_bbox_jsk;

        object_bbox_jsk.pose = test_bbox.center;
        object_bbox_jsk.dimensions = test_bbox.size;
        object_bbox_jsk.header.frame_id = "base_link";
        object_bounding_box_.publish(object_bbox_jsk);
        // std::cout<<"distance_actual: "<<distance_actual<<std::endl;
        // If it is not in collision with other objects, return the pose
        if (num_points > points_threshold)
        {              // Maybe 0 is too strict
            continue;  // right syntax?
        }
        else
        {
            // std::cout<<"push_back here"<<std::endl;
            // test_object_pose.position.x += 1;
            if (distance == 0)
            {
                pose.push_back(test_object_pose);
            }
            // std::cout<<"distance_actual: "<<distance_actual<<std::endl;
            dis_pose_map[distance_actual] = test_object_pose;
            // cout<<"num_points: "<<num_points<<endl;
            bool success = true;
            if (dis_pose_map.size() > 50 && distance != 0)
            {
                // std::cout<<dis_pose_map.size()<<std::endl;
                // std::cout<<"pose: "<<pose.size()<<std::endl;
                break;
            }
            if (pose.size() > 50 && distance == 0)
            {
                break;
            }
        }
    }
    if (distance != 0)
    {
        pose.clear();
        for (std::map<double, geometry_msgs::Pose>::iterator iter_map = dis_pose_map.begin();
             iter_map != dis_pose_map.end(); iter_map++)
        {
            // cout<<"iter_map.first: "<<iter_map->first<<endl;
            pose.push_back(iter_map->second);
            // cout<<"iter_map->second: "<<iter_map->second<<endl;
        }
    }
    // std::reverse(pose.begin(), pose.end());
    // cout<<"pose[0]: "<<pose[0]<<endl;
    return success;
}
