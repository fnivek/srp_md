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

/*  function: generate_init_trajectory
    description:
        Generates / Loads initial trajectory for Fetch, bring the gripper up in the air, and saves it
        Modification of grasploc_pick() function?
    args:
    return:
        bool (true for success)
*/
bool Act::generate_init_trajectory()
{
    // Initialize as not loaded
    bool load = false;

    // Check the parameters to see if trajectory needs to be loaded
    nh_.getParam("/load_or_plan_flgs", load);

    // Move to prepare pose (use recorded trajectory)
    moveit::planning_interface::MoveGroupInterface::Plan prep_plan;
    std::string prep_traj_name = "/Prepare_Grasploc_Pick";

    // If trajectory fails to load, start planning
    if (load && !load_trajectory(prep_plan, prep_traj_name))
    {
        ROS_INFO("Failed to load trajectory");
        load = false;
    }

    // If not loaded, generate the trajectory
    if (!load)
    {
        bool plan_success = true;
        do
        {
            std::string pose_name = "pre_grasploc_grasp";
            auto plan_result = plan(pose_name);
            plan_success = plan_result.first;
            prep_plan = plan_result.second;
        } while (plan_success && boolean_interface("replan"));

        if (!plan_success)
            return false;

        if (!record_trajectory(prep_plan, prep_traj_name))
            ROS_INFO("Unable to save the plan");
    }

    // Execute the plan, move the arm to prepare pose
    bool move_to_prep_success = (bool)move_group_.execute(prep_plan);
    if (!move_to_prep_success)
    {
        ROS_INFO("Fail to move the arm to prepare pose");
        return false;
    }
    return true;
}


/*  function: grasploc_pick
    description:
        Moves the Fetch hand to grasp position of best  grasp pose of the object
    args:
        grasp_poses (vector<geometry_msgs::Pose>)
        normals (vector<geometry_msgs::Vector3>)
        principals (vector<geometry_msgs::Vector3>)
    return:
        bool (true for success)
*/
bool Act::grasploc_pick(std::vector<geometry_msgs::Pose> grasp_poses, std::vector<geometry_msgs::Vector3> normals,
                         std::vector<geometry_msgs::Vector3> principals, bool filter)
{
    /* For RVIZ visualization */
    ros::Publisher grasp_pose_vis_pub = nh_.advertise<visualization_msgs::Marker>("grasp_pose_visualization_marker", 0);
    ros::Publisher pre_grasp_pose_vis_pub = nh_.advertise<visualization_msgs::Marker>("pre_grasp_pose_visualization_marker", 0);

    // Move to the initial grasp pose
    if (!generate_init_trajectory())
    {
        ROS_INFO("Unable to generate the initial trajectory");
        return false;
    }

    // Initialize variables to calculate pre-grasp pose
    size_t num_poses = grasp_poses.size();
    bool success = false;
    moveit::planning_interface::MoveGroupInterface::Plan pick_plan;
    geometry_msgs::Pose pre_grasp_pose;
    geometry_msgs::Pose final_grasp_pose;

    // For each candidate pose in grasp_poses,
    for (size_t i = 0; i < grasp_poses.size(); i++)
    {
        // Initialize variables
        bool plan_success = true;
        auto normal = normals[i];
        auto principal = principals[i];
        auto grasp_pose = grasp_poses[i];
        ROS_INFO("new grasp point: x=%f, y=%f, z=%f", grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);
        ROS_INFO("normal: x=%f, y=%f, z=%f", normal.x, normal.y, normal.z);

        // Get the axis of grasping
        tf::Vector3 x_axis(normal.x, normal.y, normal.z);
        x_axis = x_axis.normalize();

        // Compute offset innate to Fetch
        tf::Vector3 offset = 0.154 * x_axis;
        grasp_pose.position.x = grasp_pose.position.x + offset.x();
        grasp_pose.position.y = grasp_pose.position.y + offset.y();
        grasp_pose.position.z = grasp_pose.position.z + offset.z();

        // Compute pre-grasp pose along the normal of the gripper, based on the filter
        tf::Vector3 pre_offset;
        if (!filter)
        {
            pre_offset = 0.05 * x_axis;
        }
        else
        {
            pre_offset = 0.15 * x_axis;
        }
        pre_grasp_pose.position.x = grasp_pose.position.x + pre_offset.x();
        pre_grasp_pose.position.y = grasp_pose.position.y + pre_offset.y();
        pre_grasp_pose.position.z = grasp_pose.position.z + pre_offset.z();

        // If filter on, try checking if robot hand needs to approach from bottom, and discard this pose if so
        if (filter)
        {
            tf::Vector3 z(0, 0, 1);
            float cos_angle_to_z = x_axis.dot(z);
            if (cos_angle_to_z < 0.9063077870366499)
            {
                ROS_INFO("Pre-grasp-pose angle to z-axis is more than 25 deg");
                continue;
            }
        }

        // Orientation is exactly same for grasp and pre-grasp
        pre_grasp_pose.orientation.x = grasp_pose.orientation.x;
        pre_grasp_pose.orientation.y = grasp_pose.orientation.y;
        pre_grasp_pose.orientation.z = grasp_pose.orientation.z;
        pre_grasp_pose.orientation.w = grasp_pose.orientation.w;

        // ROS_INFO("grasp position: x=%f, y=%f, z=%f", grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);
        // ROS_INFO("grasp orientation: x=%f, y=%f, z=%f, w=%f", grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);
        // ROS_INFO("pre grasp position: x=%f, y=%f, z=%f", pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);
        // ROS_INFO("pre grasp orientation: x=%f, y=%f, z=%f, w=%f", pre_grasp_pose.orientation.x, pre_grasp_pose.orientation.y, pre_grasp_pose.orientation.z, pre_grasp_pose.orientation.w);

        // Visualize pre-grasp poses in RVIZ
        visualization_msgs::Marker pre_grasp_marker;
        pre_grasp_marker.header.frame_id = "base_link";
        pre_grasp_marker.header.stamp = ros::Time();
        pre_grasp_marker.ns = "pose_visualization";
        pre_grasp_marker.id = 0;
        pre_grasp_marker.type = visualization_msgs::Marker::ARROW;
        pre_grasp_marker.action = visualization_msgs::Marker::ADD;
        pre_grasp_marker.pose.position.x = pre_grasp_pose.position.x;
        pre_grasp_marker.pose.position.y = pre_grasp_pose.position.y;
        pre_grasp_marker.pose.position.z = pre_grasp_pose.position.z;
        pre_grasp_marker.pose.orientation.x = pre_grasp_pose.orientation.x;
        pre_grasp_marker.pose.orientation.y = pre_grasp_pose.orientation.y;
        pre_grasp_marker.pose.orientation.z = pre_grasp_pose.orientation.z;
        pre_grasp_marker.pose.orientation.w = pre_grasp_pose.orientation.w;
        pre_grasp_marker.scale.x = 0.05;
        pre_grasp_marker.scale.y = 0.05;
        pre_grasp_marker.scale.z = 0.05;
        pre_grasp_marker.color.r = 0.0f;
        pre_grasp_marker.color.g = 1.0f;
        pre_grasp_marker.color.b = 0.0f;
        pre_grasp_marker.color.a = 1.0;
        pre_grasp_marker.lifetime = ros::Duration();
        pre_grasp_pose_vis_pub.publish(pre_grasp_marker);

        // Visualize grasp poses in RVIZ
        visualization_msgs::Marker grasp_marker;
        grasp_marker.header.frame_id = "base_link";
        grasp_marker.header.stamp = ros::Time();
        grasp_marker.ns = "pose_visualization";
        grasp_marker.id = 1;
        grasp_marker.type = visualization_msgs::Marker::ARROW;
        grasp_marker.action = visualization_msgs::Marker::ADD;
        grasp_marker.pose.position.x = grasp_pose.position.x;
        grasp_marker.pose.position.y = grasp_pose.position.y;
        grasp_marker.pose.position.z = grasp_pose.position.z;
        grasp_marker.pose.orientation.x = grasp_pose.orientation.x;
        grasp_marker.pose.orientation.y = grasp_pose.orientation.y;
        grasp_marker.pose.orientation.z = grasp_pose.orientation.z;
        grasp_marker.pose.orientation.w = grasp_pose.orientation.w;
        grasp_marker.scale.x = 0.05;
        grasp_marker.scale.y = 0.05;
        grasp_marker.scale.z = 0.05;
        grasp_marker.color.r = 0.0f;
        grasp_marker.color.g = 0.0f;
        grasp_marker.color.b = 1.0f;
        grasp_marker.color.a = 1.0;
        grasp_marker.lifetime = ros::Duration();
        pre_grasp_pose_vis_pub.publish(grasp_marker);

        // Plan to the pre-grasp pose
        auto plan_result = plan(pre_grasp_pose);
        plan_success = plan_result.first;
        pick_plan = plan_result.second;

        // If planning succeeds,
        if (plan_success)
        {
            // Try to move to that position with current plan
            success = true;
            final_grasp_pose = grasp_pose;
            bool move_success = (bool)move_group_.execute(pick_plan);
            if (!move_success)
            {
                ROS_INFO("Fail to move the arm to pre-grasp pose, try another pose");
                success = false;
                continue;
            }
            else
                break;
        }
        ROS_INFO("Fail to find plan from this pre-grasp pose.");
    }

    // If not successful overall, quit
    if (!success)
    {
        ROS_INFO("Fail to find plan from the poses.");
        return false;
    }

    // If moving to pre-grasp position succeeds, report the values of the pose
    ROS_INFO("grasp position: x=%f, y=%f, z=%f", final_grasp_pose.position.x, final_grasp_pose.position.y, final_grasp_pose.position.z);
    ROS_INFO("grasp orientation: x=%f, y=%f, z=%f, w=%f", final_grasp_pose.orientation.x, final_grasp_pose.orientation.y, final_grasp_pose.orientation.z, final_grasp_pose.orientation.w);
    ROS_INFO("pre grasp position: x=%f, y=%f, z=%f", pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z);
    ROS_INFO("pre grasp orientation: x=%f, y=%f, z=%f, w=%f", pre_grasp_pose.orientation.x, pre_grasp_pose.orientation.y, pre_grasp_pose.orientation.z, pre_grasp_pose.orientation.w);

    // move to grasp position with cartesian grasp
    if (!cartesian_grasp({ pre_grasp_pose, final_grasp_pose }, 3))
    {
        ROS_INFO("Fail to cartesian grasp");
    }

    geometry_msgs::Pose lift_pose = final_grasp_pose; // !!Not doing deep copy
    lift_pose.position.z += 0.2;
    if (!cartesian_move(lift_pose, 3).first)
    {
        ROS_INFO("Fail to cartesian lift up the object");
        return false;
    }
    return true;
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