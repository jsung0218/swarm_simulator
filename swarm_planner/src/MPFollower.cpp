// ROS
#include <ros/ros.h>

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

// Parameters
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Submodules
#include <ecbs_planner.hpp>
#include <rbp_corridor.hpp>
#include <rbp_planner.hpp>
#include <rbp_publisher.hpp>

bool has_octomap = false;
bool has_path = false;
std::shared_ptr<octomap::OcTree> octree_obj;
std::vector<double> target_following; 

void octomapCallback(const octomap_msgs::Octomap& octomap_msg)
{
    if(has_octomap)
        return;

    octree_obj.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));

    has_octomap = true;
}

void poseTargetCallback(const geometry_msgs::PoseStamped& pos_msg)
{
    if(has_path)
        return;

    target_following.clear();
    target_following.push_back(pos_msg.pose.position.x);
    target_following.push_back(pos_msg.pose.position.y);
    target_following.push_back(pos_msg.pose.position.z);
}

int main(int argc, char* argv[]) {
    ROS_INFO("Follower Trajectory Planner");
    ros::init (argc, argv, "MPFollower_node");
    ros::NodeHandle nh( "~" );
    ros::Subscriber octomap_sub = nh.subscribe( "/octomap_full", 1, octomapCallback );

    // subscriber to reciever poseStamped which is assigned to Start and Goal.
    ros::Subscriber pose_target_sub = nh.subscribe( "/pose_quad/mavmaster", 1, poseTargetCallback );

    // Mission
    SwarmPlanning::Mission mission;
    if(!mission.setMission(nh)){
        return -1;
    }

    // ROS Parameters
    SwarmPlanning::Param param;
    if(!param.setROSParam(nh)){
        return -1;
    }
    param.setColor(4);
    param.setColorUse(2);

    // Submodules
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<RBPPlanner> RBPPlanner_obj;
    std::shared_ptr<ResultPublisher> resultPublisher_obj;

    // Main Loop
    ros::Rate rate(20);
    Timer timer_total;
    Timer timer_step;
    double start_time, current_time;
    while (ros::ok()) {
        if (has_octomap && !has_path) {
            timer_total.reset();

            // Build 3D Euclidean Distance Field
            timer_step.reset();
            {
                float maxDist = 1;
                octomap::point3d min_point3d(param.world_x_min, param.world_y_min, param.world_z_min);
                octomap::point3d max_point3d(param.world_x_max, param.world_y_max, param.world_z_max);
                distmap_obj.reset(new DynamicEDTOctomap(maxDist, octree_obj.get(), min_point3d, max_point3d, false));
                distmap_obj.get()->update();
            }
            timer_step.stop();
            ROS_INFO_STREAM("distmap runtime: " << timer_step.elapsedSeconds());

            // Step 1: Plan Initial Trajectory
            timer_step.reset();
            {
                initTrajPlanner_obj.reset(new ECBSPlanner(distmap_obj, mission, param));
                if (!initTrajPlanner_obj.get()->update(param.log)) {
                    return -1;
                }
            }
            timer_step.stop();
            ROS_INFO_STREAM("Initial Trajectory Planner runtime: " << timer_step.elapsedSeconds());

            // Step 2: Generate SFC, RSFC
            timer_step.reset();
            {
                corridor_obj.reset(new Corridor(initTrajPlanner_obj, distmap_obj, mission, param));
                if (!corridor_obj.get()->update(param.log)) {
                    return -1;
                }
            }
            timer_step.stop();
            ROS_INFO_STREAM("BoxGenerator runtime: " << timer_step.elapsedSeconds());

            // Step 3: Formulate QP problem and solving it to generate trajectory for quadrotor swarm
            timer_step.reset();
            {
                RBPPlanner_obj.reset(new RBPPlanner(corridor_obj, initTrajPlanner_obj, mission, param));
                if (!RBPPlanner_obj.get()->update(param.log)) {
                    // return -1;
                    mission.goalState[0] = target_following;
                    continue;
                }
            }
            timer_step.stop();
            ROS_INFO_STREAM("SwarmPlanner runtime: " << timer_step.elapsedSeconds());

            timer_total.stop();
            ROS_INFO_STREAM("Overall runtime: " << timer_total.elapsedSeconds());

            // Plot Planning Result
            resultPublisher_obj.reset(new ResultPublisher(nh, RBPPlanner_obj, corridor_obj, initTrajPlanner_obj, mission, param));
            //resultPublisher_obj->plot(param.log);

            start_time = ros::Time::now().toSec();
            has_path = true;
        }
        if(has_path) {
            // Publish Swarm Trajectory
            current_time = ros::Time::now().toSec() - start_time;
            resultPublisher_obj.get()->update(current_time);
            resultPublisher_obj.get()->publish();

            if( current_time > 1 ){
                has_path = false;
                    // Mission
                std::vector<double> temp_state;                    
                temp_state.resize(3);
                
                if( target_following.size() != 3)
                    continue;
                ROS_INFO("New Target pos : (%1.2f,%1.2f,%1.2f)", target_following[0], target_following[1], target_following[2]);
                temp_state = mission.startState[0];
                mission.startState[0] = mission.goalState[0];
                mission.goalState[0] = target_following;
            }

        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}