#include "srp_md/actions.h"

// default constructor that starts all the action clients
Act::Act(ros::NodeHandle &nh)
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
    test_plane_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("test_plane", 1000);

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
bool Act::load_trajectory(moveit::planning_interface::MoveGroupInterface::Plan &plan,
                           const std::string &trajectory_name)
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
            success = success && nh_.getParam(trajectory_name + "/index" + std::to_string(i) + "/velocities", velocities);
            success = success && nh_.getParam(trajectory_name + "/index" + std::to_string(i) + "/accelerations", accelerations);
            success = success && nh_.getParam(trajectory_name + "/index" + std::to_string(i) + "/time_from_start", time_from_start);
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
std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
Act::plan(const std::string &pose_name, int max_try)
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
Act::plan(const std::map<std::string, double> &joint_angles, int max_try)
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

std::pair<bool, moveit::planning_interface::MoveGroupInterface::Plan>
Act::plan(const geometry_msgs::Pose &pose, int max_try)
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
bool Act::boolean_interface(const std::string &action)
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
bool Act::record_trajectory(const moveit::planning_interface::MoveGroupInterface::Plan &plan, std::string name)
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
                           pcl::PCLPointCloud2& pcl_out_pc)
{
    pcl::CropBox<pcl::PCLPointCloud2> filt;
    filt.setInputCloud(pcl_in_pc);
    // Generate the max and min points of the bounding box
    Eigen::Affine3f tf =
        Eigen::Translation<float, 3>(crop_box.center.position.x, crop_box.center.position.y, crop_box.center.position.z)
        * Eigen::Quaternionf(crop_box.center.orientation.w, crop_box.center.orientation.x,
                             crop_box.center.orientation.y, crop_box.center.orientation.z);
    Eigen::Vector4f max_pt(crop_box.size.x / 2, crop_box.size.y / 2, crop_box.size.z / 2, 1);
    Eigen::Vector4f min_pt(-crop_box.size.x / 2, -crop_box.size.y / 2, -crop_box.size.z / 2, 1);
    filt.setMax(max_pt);
    filt.setMin(min_pt);
    filt.setTransform(tf.inverse());
    filt.filter(pcl_out_pc);
}

// Pont clouds
void Act::crop_box_filt_pc(const sensor_msgs::PointCloud2& in_pc, const vision_msgs::BoundingBox3D& crop_box,
                           sensor_msgs::PointCloud2& out_pc)
{
    // Convert to pcl point cloud 2
    pcl::PCLPointCloud2::Ptr pcl_in_pc(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2 pcl_out_pc;
    pcl_conversions::toPCL(in_pc, *pcl_in_pc);
    // Make a crop box filter
    crop_box_filt_pcl_pc(pcl_in_pc, crop_box, pcl_out_pc);

    // Move output data over
    pcl_conversions::moveFromPCL(pcl_out_pc, out_pc);
}

void Act::transform_pc(const sensor_msgs::PointCloud2& in_pc, std::string frame_id,
                       sensor_msgs::PointCloud2& out_pc)
{
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
bool Act::cartesian_grasp(const std::vector<geometry_msgs::Pose> &waypoints, int max_try /* = 3 */)
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
Act::cartesian_move(const geometry_msgs::Pose &end_pose, int max_try /* = 3 */)
{
    std::vector<geometry_msgs::Pose> waypoints = { end_pose };
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
Act::cartesian_move(const std::vector<geometry_msgs::Pose> &waypoints, int max_try /* = 3 */)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (waypoints.empty())
        return { false, plan };
    if (max_try <= 0)
        max_try = 1;

    std::pair<bool, moveit_msgs::RobotTrajectory> plan_result = cartesian_plan(waypoints, max_try);
    if (!plan_result.first)
        return { false, plan };

    moveit_msgs::RobotTrajectory &trajectory_msg = plan_result.second;

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
std::pair<bool, moveit_msgs::RobotTrajectory> Act::cartesian_plan(const geometry_msgs::Pose &end_pose,
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
std::pair<bool, moveit_msgs::RobotTrajectory> Act::cartesian_plan(const std::vector<geometry_msgs::Pose> &waypoints,
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
bool Act::move(const std::string &pose_name, int max_try /* = 1 */)
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
bool Act::move(const std::map<std::string, double> &joint_angles, int max_try /* = 1 */)
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
bool Act::move(const geometry_msgs::Pose &pose, int max_try /* = 1 */)
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
bool Act::relative_move(const geometry_msgs::Transform &pose_diff, int max_try /* = 1 */)
{
    geometry_msgs::Pose current_pose = move_group_.getCurrentPose().pose;
    tf2::Transform current_pose_tf;
    tf2::fromMsg(current_pose, current_pose_tf);

    tf2::Transform pose_diff_tf;
    tf2::fromMsg(pose_diff, pose_diff_tf);

    tf2::Transform computed_tf(pose_diff_tf.getRotation() * current_pose_tf.getRotation(),
        pose_diff_tf.getOrigin() + current_pose_tf.getOrigin());
    geometry_msgs::Pose computed;
    tf2::toMsg(computed_tf, computed);

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
geometry_msgs::Pose GetStackPose(geometry_msgs::Pose bot_pose, geometry_msgs::Vector3 bot_dim, geometry_msgs::Vector3 dim)
{
    //convert input pose from geometry_msgs to tf2
    tf2::Transform bot_pose_tf;
    tf2::fromMsg(bot_pose, bot_pose_tf);

    //convert input dimension from geometry_msgs to tf2
    tf2::Vector3 bot_dim_tf(0,0,0);
    tf2::Vector3 dim_tf(0,0,0);
    tf2::convert(bot_dim, bot_dim_tf);
    tf2::convert(dim, dim_tf);

    //create coefficient the dimension of object to multiply by to get points coordinate in its own frame
    float length_coeff[8] = {0.5, 0.5, 0.5, 0.5, -0.5, -0.5, -0.5, -0.5};
    float width_coeff[8] = {0.5, 0.5, -0.5, -0.5, 0.5, 0.5, -0.5, -0.5};
    float height_coeff[8] = {0.5, -0.5, 0.5, -0.5, 0.5, -0.5, 0.5, -0.5};

    //initialize vector to store points
    std::vector<tf2::Vector3> bot_points_botframe;
    std::vector<tf2::Vector3> bot_points_world;

    //loop through all points to find highest point
    float max_z_value = -1;
    int max_index = -1;
    std::cout << sizeof(length_coeff) / sizeof(*length_coeff) << std::endl;
    for (int i = 0; i < sizeof(length_coeff) / sizeof(*length_coeff); i++) {
        bot_points_botframe.push_back(tf2::Vector3(bot_dim_tf.x() * length_coeff[i],
            bot_dim_tf.y() * width_coeff[i], bot_dim_tf.z() * height_coeff[i]));

        bot_points_world.push_back(bot_pose_tf * bot_points_botframe[i]);
        if (bot_points_world[i].z() > max_z_value) {
            max_z_value = bot_points_world[i].z();
            max_index = i;
        }
    }
    //get the radius and pose of upper object
    tf2::Vector3 origin_bot = bot_pose_tf.getOrigin();
    float z_radius = sqrt(dim_tf.x() * dim_tf.x() + dim_tf.y() * dim_tf.y() + dim_tf.z() * dim_tf.z()) / 2;
    //keep same orientation as bottom one and add half of radius to z axis
    tf2::Transform pose_upper_tf(tf2::Quaternion(0,0,0,1),
        tf2::Vector3(origin_bot.x(), origin_bot.y(), bot_points_world[max_index].z() + z_radius));

    //convert output to geometry_msgs form
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
bool Act::get_table(const sensor_msgs::PointCloud2::ConstPtr& points, std::vector<vision_msgs::BoundingBox3D>& plane_bboxes)
{
    // Initialize variables
    sensor_msgs::PointCloud2::Ptr points_tf(new sensor_msgs::PointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    // Transform the pointcloud to base frame
    transform_pc(*points, "base_link", *points_tf);

    // Convert to PCL PointCloudc
    pcl::PCLPointCloud2::Ptr points_pcl2(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_pcl (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::toPCL(*points_tf, *points_pcl2);
    pcl::fromPCLPointCloud2 (*points_pcl2, *points_pcl);

    // Do parametric segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (0.01);

    // Creating filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // Save the original size of point cloud
    int i = 0, nr_points = (int) points_pcl->points.size ();

    //std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> plane_cloud_vector;
    std::vector<tf2::Vector3> sizes_tf_vector;
    std::vector<tf2::Transform> poses_tf_vector;
    while (points_pcl->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (points_pcl);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
          break;
        }
        //plane_cloud_vector.push_back(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
        // Extract the inliers
        extract.setInputCloud (points_pcl);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        sensor_msgs::PointCloud2 points_cloud_msg;
        pcl::toROSMsg(*cloud_p,points_cloud_msg );
        test_plane_pub_.publish(points_cloud_msg);

        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points, out of " << nr_points << " total points." << std::endl;
        for (auto&& value : coefficients->values) {
            std::cerr << "Coefficient: " << value << std::endl;
        }
        // std::stringstream ss;
        //ss << "table_scene_lms400_plane_" << i << ".pcd";
        //writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        points_pcl.swap (cloud_f);
        i++;


        // Normalize the coefficients A, B and C from [A, B, C, D] and take dot product with [0, 0, 1] (essentially just C normalized)
        // If the value is greater than cos(5deg), get the table by getting min_x, min_y, max_x, max_y for bounding box and averaging them
        // for centroid. Width = (max_x - min_x), Height = (max_y - min_y). Make pose with zero quaternion and (avg_x, avg_y, avg_z) and append
        // to poses vector. Make Vector3 with (width, height, degree) and save to sizes vector
        float norm_div = sqrt(coefficients->values[0] * coefficients->values[0] +
                              coefficients->values[1] * coefficients->values[1] +
                              coefficients->values[2] * coefficients->values[2]);
        float dotproduct = coefficients->values[2] / norm_div;
        if (dotproduct >= cos(5 * PI / 180)) {
            pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
            feature_extractor.setInputCloud (cloud_p);
            feature_extractor.compute ();
            pcl::PointXYZ min_point_AABB;
            pcl::PointXYZ max_point_AABB;
            pcl::PointXYZ position_AABB;
            //Eigen::Matrix3f rotational_matrix_AABB;
            feature_extractor.getAABB (min_point_AABB, max_point_AABB);
            sizes_tf_vector.push_back(tf2::Vector3(max_point_AABB.x - min_point_AABB.x,
                                                   max_point_AABB.y - min_point_AABB.y,
                                                   acos(dotproduct)));
            poses_tf_vector.push_back(tf2::Transform(tf2::Quaternion(0,0,0,1),
                                      tf2::Vector3((min_point_AABB.x + max_point_AABB.x) / 2,
                                                   (min_point_AABB.y + max_point_AABB.y) / 2,
                                                   (min_point_AABB.z + max_point_AABB.z) / 2)
                                                    )
                                     );
        } else {
            //plane_cloud_vector.pop_back();
        }
    }
    //rviz
    std::vector<float> plane_area_copy;
    std::vector<float> plane_area;
    int plane_index;
    for (auto&& size : sizes_tf_vector) {
        plane_area.push_back(size.x() * size.y());
        plane_area_copy.push_back(size.x() * size.y());
    }

    for(int i = 1; i < plane_area.size(); i++){
        float key=plane_area[i];
        int j=i-1;
        while((j>=0) && (key>plane_area[j])){
            plane_area[j+1]=plane_area[j];
            j--;
        }
        plane_area[j+1]=key;
    }
    std::vector<float>::iterator iter;
    for (int i = 0; i < plane_area.size(); i++) {
        iter = find(plane_area_copy.begin(), plane_area_copy.end(), plane_area[i]);

        plane_index = std::distance(plane_area_copy.begin(), iter);
        std::cerr<<"area: "<<"index: "<<plane_index<<" "<<plane_area[i]<<std::endl;
        geometry_msgs::Vector3 size_msg;
        geometry_msgs::Pose pose_msg;
        tf2::convert(sizes_tf_vector[plane_index], size_msg);
        tf2::toMsg(poses_tf_vector[plane_index], pose_msg);
        std::cerr<<"size_msg: "<<"index: "<<plane_index<<std::endl<<size_msg<<std::endl;
        std::cerr<<"pose_msg: "<<"index: "<<plane_index<<std::endl<<pose_msg<<std::endl;
        vision_msgs::BoundingBox3D plane_bbox;
        plane_bbox.center = pose_msg;
        plane_bbox.size = size_msg;
        plane_bboxes.push_back(plane_bbox);
    }
    std::cout<<"all planes: "<<plane_bboxes.size()<<std::endl;
    bool success = true;
    return success;
}

/*  function: free_space_finder()
    description:
        finds a free space of the table
    args:

    return:

*/



bool Act::free_space_finder(const sensor_msgs::PointCloud2::ConstPtr& points, const vision_msgs::BoundingBox3D& plane_bbox,
    const vision_msgs::BoundingBox3D& obj_bbox, geometry_msgs::Pose& pose)
{
    // Initialize variables
    sensor_msgs::PointCloud2::Ptr points_tf(new sensor_msgs::PointCloud2);
    geometry_msgs::Pose plane_pose = plane_bbox.center;
    geometry_msgs::Vector3 plane_size = plane_bbox.size;
    bool success = false;

    // Transform the pointcloud to base frame
    transform_pc(*points, "base_link", *points_tf);

    // Convert to PCL PointCloudc
    pcl::PCLPointCloud2::Ptr points_pcl2(new pcl::PCLPointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_pcl_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr points_pcl2_filtered (new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*points_tf, *points_pcl2);

    // Crop out the plane
    crop_box_filt_pcl_pc(points_pcl2, test_bbxox, *points_pcl2_filtered);

    // Initialize structures for random sampling
    static std::default_random_engine e(time(0));
    static std::normal_distribution<double> n(MU,SIGMA);
    static std::uniform_real_distribution<double> dis(1.0, 2.0);

    // Might want to cap this at limit 1000 or sth
    while (true) {

        // Generate random pose on the table
        double x_random, y_random;
        x_random = ?;
        y_random = ?;
        geometry_msgs::Pose test_object_pose = plane_pose;
        test_object_pose.x = test_object_pose.x + x_random - plane_size.x / 2;
        test_object_pose.y = test_object_pose.y + y_random - plane_size.y / 2;
        test_object_pose.z = ?; // May need to add the bounding box height / 2
        test_object_pose.orientation = ?; // random rotation?
        

        vision_msgs::BoundingBox3D test_bbxox;
        test_bbxox.size = obj_bbox.size;
        test_bbxox.center = test_object_pose;
        crop_box_filt_pcl_pc(points_pcl2, test_bbxox, *points_pcl2_filtered);
        pcl::fromPCLPointCloud2 (*points_pcl2_filtered, *points_pcl_filtered); // Is this step needed?

        // If it is not in collision with other objects, return the pose
        if (points_pcl_filtered->points.size() > 0) { // Maybe 0 is too strict
            continue; // right syntax?
        } else {
            pose = test_object_pose;
            bool success = true;
            break;
        }
    }
    return success;
}