#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

// Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

// ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Parameters
#include <param.hpp>
#include <mission.hpp>
#include <timer.hpp>

// Submodules
#include <ecbs_planner.hpp>
#include <rbp_corridor.hpp>
#include <rbp_planner.hpp>
#include <rbp_publisher.hpp>

#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>

#include "bezier_base.h"

#include <sp_const.hpp>



using namespace std;
using namespace Eigen;
using namespace pcl;

ros::Publisher _vis_ctrl_pts_pub;
ros::Publisher _vis_commit_target_pub;
ros::Publisher _traj_pub;
ros::Publisher _vis_target_points;
ros::Publisher _vis_traj_points;
ros::Publisher _vis_commit_traj_points;
ros::Publisher _vis_stop_traj_points;
ros::Subscriber _odom_sub;
ros::Subscriber _dest_pts_sub;
ros::Subscriber _map_sub;
ros::Subscriber _octomap_sub;
ros::Timer _planning_timer;

SwarmPlanning::Param _param;
SwarmPlanning::Mission _mission;
    

/*  parameters read from lauch file  */
double _vel_max, _acc_max, _vel_mean, _acc_mean, _eps;
double _x_l, _x_h, _y_l, _y_h, _z_l, _z_h;  // For random map simulation : map boundary
double _refine_portion, _path_find_limit, _sample_portion, _goal_portion;
double _safety_margin, _search_margin, _max_radius, _sensing_range, _planning_rate, _stop_time, _time_commit;
int    _minimize_order, _poly_order_min, _poly_order_max, _max_samples;
bool   _use_preset_goal, _is_limit_vel, _is_limit_acc, _is_print;

Vector3d _start_pos, _start_vel, _start_acc, _commit_target, _end_pos;
double _time_limit_1, _time_limit_2;
nav_msgs::Odometry _odom;

int _traj_id = 0;
int _segment_num = 0;
vector<int> _poly_orderList;
ros::Time _odom_time;
ros::Time _start_time = ros::TIME_MAX;
ros::Time _rcv_odom_time = ros::TIME_MAX;
ros::Time _plan_traj_time = ros::TIME_MAX;

bool _is_traj_exist = false;
bool _is_target_arrive = false;
bool _is_target_receive = false;
bool _is_has_map = false;

MatrixXd _Path;
MatrixXd _PolyCoeff;
VectorXd _Time;
SFC_t SFC; // safe flight corridors to avoid obstacles

vector<MatrixXd> _FMList;
vector<VectorXd> _CList, _CvList, _CaList;
sensor_msgs::PointCloud2 traj_pts, target_pts, traj_commit_pts, traj_stop_pts;
PointCloud<PointXYZ> traj_pts_pcd, target_pts_pcd, traj_commit_pts_pcd, traj_stop_pts_pcd;
visualization_msgs::MarkerArray path_vis;
visualization_msgs::MarkerArray ctrlPt_vis;

// Submodules
std::shared_ptr<octomap::OcTree> octree_obj;
std::shared_ptr<DynamicEDTOctomap> distmap_obj;
std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
std::shared_ptr<Corridor> corridor_obj;
std::shared_ptr<RBPPlanner> RBPPlanner_obj;

int trajGeneration( double time_odom_delay);
Vector3d getCommitedTarget();
quadrotor_msgs::PolynomialTrajectory getBezierTraj();

void getPosFromBezier(const MatrixXd& polyCoeff, const double& t_now, Vector3d& ret);
void getStateFromBezier(const MatrixXd& polyCoeff, const double& t_now, VectorXd& ret);


void octomapCallback(const octomap_msgs::Octomap& octomap_msg)
{
    if(_is_has_map)
        return;
    octree_obj.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));

    if( !_is_has_map)
    {
        float maxDist = 1;
        octomap::point3d min_point3d(_param.world_x_min, _param.world_y_min, _param.world_z_min);
        octomap::point3d max_point3d(_param.world_x_max, _param.world_y_max, _param.world_z_max);
        distmap_obj.reset(new DynamicEDTOctomap(maxDist, octree_obj.get(), min_point3d, max_point3d, false));
        distmap_obj.get()->update();
        _is_has_map = true;
        ROS_INFO(" Octomap ia ready");
    }
}


void rcvWaypointsCallBack(const nav_msgs::Path & wp)
{   
    if(wp.poses[0].pose.position.z < 0.0)
      return;

    _end_pos(0) = wp.poses[0].pose.position.x;
    _end_pos(1) = wp.poses[0].pose.position.y;
    _end_pos(2) = wp.poses[0].pose.position.z;

    _is_target_receive  = true;
    _is_target_arrive   = false;
    _is_traj_exist      = false;
}

void rcvOdometryCallBack(const nav_msgs::Odometry odom)
{
    _odom = odom;
    _odom_time = odom.header.stamp;

    std::vector<double> start_pose; 
    _start_pos(0) = _odom.pose.pose.position.x;
    _start_pos(1) = _odom.pose.pose.position.y;
    _start_pos(2) = _odom.pose.pose.position.z;
    _start_vel(0) = _odom.twist.twist.linear.x;
    _start_vel(1) = _odom.twist.twist.linear.y;
    _start_vel(2) = _odom.twist.twist.linear.z;
    _start_acc(0) = _odom.twist.twist.angular.x;
    _start_acc(1) = _odom.twist.twist.angular.y;
    _start_acc(2) = _odom.twist.twist.angular.z;

    start_pose.push_back(_start_pos(0));
    start_pose.push_back(_start_pos(1));
    start_pose.push_back(_start_pos(2));
    start_pose.push_back(_start_vel(0));
    start_pose.push_back(_start_vel(1));
    start_pose.push_back(_start_vel(2));
    start_pose.push_back(_start_acc(0));
    start_pose.push_back(_start_acc(1));
    start_pose.push_back(_start_acc(2));

    _mission.startState[0] = start_pose;

    _rcv_odom_time = ros::Time::now();
    // _rrtPathPlaner.setStartPt(_start_pos, _end_pos);  

    const static auto getDis = []( const Vector3d u, const Vector3d v ){
        const static auto sq = [] (double && val){ 
            return val * val;
        };

        return sqrt( sq(v[0] - u[0]) + sq(v[1] - u[1]) + sq(v[2] - u[2])  );
    };
    static int gCount = 0;
    static ros::Time gFirstTime = ros::Time::now();
    double dist = getDis(_start_pos, _commit_target);
    double duration_time;
    if(gCount++%50 == 0 )
    {
        duration_time = ros::Time::now().toSec() - gFirstTime.toSec();
        ROS_INFO("[My] (%1.2fsec) Dist : %1.2f (%1.2f,%1.2f,%1.2f), (%1.2f,%1.2f,%1.2f)", 
                    duration_time, dist, _start_pos.x(), _start_pos.y(), _start_pos.z(), 
                    _commit_target.x(), _commit_target.y(), _commit_target.z());
                            
    }
    
    if( _is_traj_exist && dist < _eps )
    {
        ROS_WARN("[My] Target arrived..... ");
        _is_target_arrive = true;
    }
}

void rcvPointCloudCallBack(const sensor_msgs::PointCloud2 & pointcloud_map )
{   
    PointCloud<PointXYZ> cloud_input;      
    pcl::fromROSMsg(pointcloud_map, cloud_input);

    if(cloud_input.points.size() == 0) return;

    _is_has_map = true;
    // _rrtPathPlaner.setInput(cloud_input);
    /*
    if(checkSafeTrajectory(_stop_time))
    {
        ROS_WARN("[Demo] Collision Occur, Stop");
        quadrotor_msgs::PolynomialTrajectory traj;
        traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
        _traj_pub.publish(traj);
        _is_traj_exist = false;  
    }
    */
}


int trajGeneration(double time_odom_delay)
{           
    std::vector<double> start_pose; 
    std::vector<double> end_pose; 

    start_pose.clear();
    end_pose.clear();
    
    if(_is_traj_exist){
        double time_est_opt = 0.03;      
        double t_s =  (_odom_time - _start_time).toSec() + time_est_opt + time_odom_delay;     

        VectorXd state;
        getStateFromBezier(_PolyCoeff, t_s, state);

        for(int i = 0; i < 9; i++ )
        {
            start_pose.push_back(state(i) );
        }
    }
    else{   
        start_pose.push_back( _start_pos.x());
        start_pose.push_back( _start_pos.y());
        start_pose.push_back( _start_pos.z());
        start_pose.push_back( _start_vel.x());
        start_pose.push_back( _start_vel.y());
        start_pose.push_back( _start_vel.z());
        start_pose.push_back( _start_acc.x());
        start_pose.push_back( _start_acc.y());
        start_pose.push_back( _start_acc.z());
    }
    end_pose.push_back(_end_pos.x());
    end_pose.push_back(_end_pos.y());
    end_pose.push_back(_end_pos.z());
    end_pose.push_back(0);
    end_pose.push_back(0);
    end_pose.push_back(0);
    end_pose.push_back(0);
    end_pose.push_back(0);
    end_pose.push_back(0);

    _mission.startState[0] = start_pose;
    _mission.goalState[0] = end_pose;

    // Step 1: Plan Initial Trajectory
    initTrajPlanner_obj.reset(new ECBSPlanner(distmap_obj, _mission, _param));
    if (!initTrajPlanner_obj.get()->update(_param.log)) {
        return -1;
    }
    ROS_WARN("Initial Trajectory Planner runtime: "); //  << timer_step.elapsedSeconds());

     int isize= initTrajPlanner_obj.get()->initTraj[0].size();
    for(int i=0;i<isize;i++)
    {
        octomap::point3d pt = initTrajPlanner_obj.get()->initTraj[0][i];
        ROS_WARN("Traj : [%d] : (%1.2f, %1.2f,%1.2f)", i, pt.x(), pt.y(), pt.z());
    }

    // Step 2: Generate SFC
    corridor_obj.reset(new Corridor(initTrajPlanner_obj, distmap_obj, _mission, _param));
    if (!corridor_obj.get()->update(_param.log)) {
        return -1;
    }

    ROS_INFO_STREAM("BoxGenerator runtime: "); // << timer_step.elapsedSeconds());

    // Step 3: Formulate QP problem and solving it to generate trajectory for quadrotor swarm

    ros::Time time_1 = ros::Time::now();
    RBPPlanner_obj.reset(new RBPPlanner(corridor_obj, initTrajPlanner_obj, _mission, _param));
    if (!RBPPlanner_obj.get()->update(_param.log)) {
        return -1;
    }
    _start_time = _odom_time + ros::Duration(ros::Time::now() - _rcv_odom_time);
    ros::Time time_2 = ros::Time::now();
    ROS_WARN("[Demo] Time in traj generation is %f", (time_2 - time_1).toSec());
    _segment_num = RBPPlanner_obj->msgs_traj_info.data.size() - 2 - 1;

    _Time.resize(_segment_num+1);
    for(int m = 0; m < _segment_num+1; m++){
        _Time(m) = RBPPlanner_obj->msgs_traj_info.data.at(m + 2);
        ROS_WARN("[Demo] %d, Time : %1.2f", m, _Time(m));
    }

    _poly_orderList.clear();
    
    for(int i =0; i < _segment_num; i ++ )
    {  
        _poly_orderList.push_back( 5 );
    }

    float rows = RBPPlanner_obj->msgs_traj_coef[0].layout.dim.at(0).size;
    float cols = RBPPlanner_obj->msgs_traj_coef[0].layout.dim.at(1).size;
    std::vector<double> data = RBPPlanner_obj->msgs_traj_coef[0].data;
    _PolyCoeff = Eigen::Map<Eigen::MatrixXd>(data.data(), rows, cols);
   
  
    quadrotor_msgs::PolynomialTrajectory traj;
    if(_PolyCoeff.rows() == 3 && _PolyCoeff.cols() == 3){
        ROS_WARN("[Demo] Cannot find a feasible and optimal solution, somthing wrong with the mosek solver ... ");
        traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
        _traj_pub.publish(traj);
        _is_traj_exist     = false;
        _is_target_receive = false;
        return -1;
    }
    else{
        
        if (_segment_num == 0)
        {
            ROS_WARN("[Demo] Trajectory segment is zero");
            traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
            _traj_pub.publish(traj);
            _is_traj_exist     = false;
            _is_target_receive = false;
            return -1;
        }
        ROS_WARN("[Demo] Trajectory generated successed");
        traj = getBezierTraj();
        _traj_pub.publish(traj);
        _is_traj_exist = true;
    }

    _traj_id ++;
    return 1;
}

bool checkEndOfCommitedTraj()
{     
    if(_is_target_arrive){     
      _is_target_arrive = false;
      return true;
    }
    else return false;
}

Eigen::Vector3d getCommitedTarget()
{
    double t_s = _time_commit;      

    int idx;
    for (idx = 0; idx < _segment_num; ++idx){
        if (t_s > _Time(idx) && idx + 1 < _segment_num)
        {
            break;
        }
    }          

    double t_start = t_s;
    Eigen::Vector3d coord_t;
    getPosFromBezier(_PolyCoeff, t_start, coord_t);

    // coord_t *= _Time(idx);
    target_pts_pcd.clear();
    target_pts_pcd.points.push_back(PointXYZ(coord_t(0), coord_t(1), coord_t(2)));    
    target_pts_pcd.width = target_pts_pcd.points.size();
    target_pts_pcd.height = 1;
    target_pts_pcd.is_dense = true;

    pcl::toROSMsg(target_pts_pcd, target_pts);
    target_pts.header.frame_id = "map";
    _vis_target_points.publish(target_pts);

    return coord_t;
}

void planInitialTraj()
{
    // _rrtPathPlaner.reset();
    // _rrtPathPlaner.setPt( _start_pos, _end_pos, _x_l, _x_h, _y_l, _y_h, _z_l, _z_h, _sensing_range, _max_samples, _sample_portion, _goal_portion );

    ros::Time timeBef = ros::Time::now();
    // Step 1: Plan Initial Trajectory
    initTrajPlanner_obj.reset(new ECBSPlanner(distmap_obj, _mission, _param));
    if (!initTrajPlanner_obj.get()->update(_param.log)) {
        return;
    }

    int isize= initTrajPlanner_obj.get()->initTraj[0].size();
    for(int i=0;i<isize;i++)
    {
        octomap::point3d pt = initTrajPlanner_obj.get()->initTraj[0][i];
        ROS_WARN("Traj : [%d] : (%1.2f, %1.2f,%1.2f)", i, pt.x(), pt.y(), pt.z());
    }
    ros::Time timeAft = ros::Time::now();
    
    //_rrtPathPlaner.SafeRegionExpansion(_path_find_limit);
    
    double _path_time = (timeAft-timeBef).toSec();

    //tie(_Path, _Radius) = _rrtPathPlaner.getPath();
    if( initTrajPlanner_obj->initTraj.size() == 0  ){
        ROS_WARN("[Demo] Can't find a path, mission stall, please reset the target");
        quadrotor_msgs::PolynomialTrajectory traj;
        traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE;
        _traj_pub.publish(traj);
        _is_traj_exist = false;
        _is_target_receive = false;
    }
    else{ // Path finding succeed .
        if( trajGeneration( _path_time ) == 1 ){   
            _commit_target = getCommitedTarget();
            // _rrtPathPlaner.resetRoot(_commit_target);
            // visBezierTrajectory(_PolyCoeff);
            // visCommitTraj(_PolyCoeff);
            // visCtrlPoint( _PolyCoeff );
            // visFlightCorridor(_Path, _Radius);
            ROS_INFO(" [My] planInitialTraj:Commited Target : (%1.2f, %1.2f, %1.2f)", _commit_target(0), _commit_target(1), _commit_target(2));
            
        }
    }

    //visRrt(_rrtPathPlaner.getTree());
}

void planIncrementalTraj()
{     
    // if(_rrtPathPlaner.getGlobalNaviStatus() == true){
    //   visRrt(_rrtPathPlaner.getTree());
    //   return;
    // }
    static double first_time = ros::Time::now().toSec();


    double cur_time = ros::Time::now().toSec() - first_time;
    if( cur_time < 3)
    {
        return;
    }
    else
    {
        first_time = ros::Time::now().toSec();
    }


    // Step 1: Plan Initial Trajectory
    initTrajPlanner_obj.reset(new ECBSPlanner(distmap_obj, _mission, _param));
    if (!initTrajPlanner_obj.get()->update(_param.log)) {
        return;
    }
    
    //_rrtPathPlaner.SafeRegionExpansion(_path_find_limit);

    

    if( checkEndOfCommitedTraj() ) {   // arrive at the end of the commited trajectory.
        //if( !_rrtPathPlaner.getPathExistStatus() ){ // no feasible path exists
        if( initTrajPlanner_obj->initTraj.size() == 0  ){
            ROS_WARN("[Demo] reach commited target but no path exists, waiting for a path");
            _is_traj_exist = false;  // continue to fly 
            return;
        }
        else{   
            //visFlightCorridor( _Path, _Radius);
            _plan_traj_time = ros::Time::now();
            if( trajGeneration( (_plan_traj_time - _rcv_odom_time).toSec() ) == 1) { // Generate a new trajectory sucessfully.
                _commit_target = getCommitedTarget();   
                // _rrtPathPlaner.resetRoot(_commit_target);  

                ros::Time time_1 = ros::Time::now();
                // visBezierTrajectory(_PolyCoeff);
                // visCommitTraj(_PolyCoeff);
                // visCtrlPoint (_PolyCoeff);
                 ROS_INFO(" [My] planIncrementalTraj:Commited Target : (%1.2f, %1.2f, %1.2f)", _commit_target(0), _commit_target(1), _commit_target(2));
            }
        }
    }
    else{   // continue to refine the uncommited trajectory
        //ROS_WARN( " [My] Add sampling to refine");
        // _rrtPathPlaner.SafeRegionRefine  ( _time_limit_1 ); // add samples to the tree
        // _rrtPathPlaner.SafeRegionEvaluate( _time_limit_2 ); // ensure that the path is collision-free

        //if(_rrtPathPlaner.getPathExistStatus() == true){ 
            // tie(_Path, _Radius) = _rrtPathPlaner.getPath();
            // visFlightCorridor(_Path, _Radius);
            //ROS_INFO(" [My] todo : continue to refine the uncommited trajectory ");

        // visRrt(_rrtPathPlaner.getTree()); 
    }
}

void planningCallBack(const ros::TimerEvent& event)
{
    if( !_is_target_receive || !_is_has_map ) 
        return;

    if( !_is_traj_exist ) planInitialTraj();
    else planIncrementalTraj();  
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Master_Following_node");
    ros::NodeHandle node_handle("~");

    node_handle.param("planParam/plan_rate",       _planning_rate,   10.0);      
    node_handle.param("planParam/safety_margin",   _safety_margin,   0.65);
    node_handle.param("planParam/search_margin",   _search_margin,   0.35);
    node_handle.param("planParam/max_radius",      _max_radius,      10.0);
    node_handle.param("planParam/sensing_range",   _sensing_range,   10.0);     
    node_handle.param("planParam/refine_portion",  _refine_portion,  0.80);     
    node_handle.param("planParam/sample_portion",  _sample_portion,  0.25);     // the ratio to generate samples inside the map range
    node_handle.param("planParam/goal_portion",    _goal_portion,    0.05);     // the ratio to generate samples on the goal
    node_handle.param("planParam/path_find_limit", _path_find_limit, 0.05);     
    node_handle.param("planParam/max_samples",     _max_samples,     3000);     
    node_handle.param("planParam/stop_horizon",    _stop_time,       0.50);     
    node_handle.param("planParam/commitTime",      _time_commit,      1.0);

    node_handle.param("init_x",        _start_pos(0),       0.0);
    node_handle.param("init_y",        _start_pos(1),       0.0);
    node_handle.param("init_z",        _start_pos(2),       0.0);
    node_handle.param("demoParam/target_x",        _end_pos(0),       0.0);
    node_handle.param("demoParam/target_y",        _end_pos(1),       0.0);
    node_handle.param("demoParam/target_z",        _end_pos(2),       0.0);
    node_handle.param("demoParam/goal_input",      _use_preset_goal, true);
    node_handle.param("demoParam/is_limit_vel",    _is_limit_vel,    true);
    node_handle.param("demoParam/is_limit_acc",    _is_limit_acc,    true);
    node_handle.param("demoParam/is_print",        _is_print,        true);

    node_handle.param("dynamic/vec",       _vel_mean, 2.0);
    node_handle.param("dynamic/acc",       _acc_mean, 1.0);
    node_handle.param("dynamic/max_vec",   _vel_max,  3.0);
    node_handle.param("dynamic/max_acc",   _acc_max,  1.5);

    node_handle.param("mapBoundary/lower_x", _x_l,  -50.0);
    node_handle.param("mapBoundary/upper_x", _x_h,   50.0);
    node_handle.param("mapBoundary/lower_y", _y_l,  -50.0);
    node_handle.param("mapBoundary/upper_y", _y_h,   50.0);
    node_handle.param("mapBoundary/lower_z", _z_l,    0.0);
    node_handle.param("mapBoundary/upper_z", _z_h,    3.0);

    node_handle.param("optimization/poly_order_min", _poly_order_min,  5);
    node_handle.param("optimization/poly_order_max", _poly_order_max, 5);
    node_handle.param("optimization/minimize_order", _minimize_order,  5);

    if(_use_preset_goal)
    {   
        ROS_WARN("[Demo] Demo using preset target");
        ROS_WARN("[Demo] Warming up ... ");

        sleep(2.0);
        _is_target_receive  = true;
        _is_target_arrive   = false;
        _is_traj_exist      = false;

    }

    if(!_mission.setMission(node_handle)){
        return -1;
    }
    
    if(!_param.setROSParam(node_handle)){
        return -1;
    }

    std::vector<double> start_pose; 
    std::vector<double> end_pose;
    
    start_pose.push_back(_start_pos(0));
    start_pose.push_back(_start_pos(1));
    start_pose.push_back(_start_pos(2));
    start_pose.push_back(0);
    start_pose.push_back(0);
    start_pose.push_back(0);
    start_pose.push_back(0);
    start_pose.push_back(0);
    start_pose.push_back(0);

    end_pose.push_back(_end_pos.x());
    end_pose.push_back(_end_pos.y());
    end_pose.push_back(_end_pos.z());
    end_pose.push_back(0);
    end_pose.push_back(0);
    end_pose.push_back(0);
    end_pose.push_back(0);
    end_pose.push_back(0);
    end_pose.push_back(0);

    _mission.startState[0] = start_pose;
    _mission.goalState[0] = end_pose;


    _mission.startState[0] = start_pose;


    Bernstein _bernstein;
    if(_bernstein.setParam(_poly_order_min, _poly_order_max, _minimize_order) == -1){
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");
        exit(EXIT_FAILURE);
    }


    _eps = 0.25; 

    _FMList  = _bernstein.getFM();
    _CList   = _bernstein.getC();
    _CvList  = _bernstein.getC_v();
    _CaList  = _bernstein.getC_a();

    _time_limit_1 =      _refine_portion  * 1.0 / _planning_rate;
    _time_limit_2 = (1 - _refine_portion) * 1.0 / _planning_rate; 
          
    // subcribed msgs
    _dest_pts_sub = node_handle.subscribe( "waypoints",  1, rcvWaypointsCallBack );
    _map_sub      = node_handle.subscribe( "PointCloud", 1, rcvPointCloudCallBack);
    _odom_sub     = node_handle.subscribe( "odometry",   1, rcvOdometryCallBack );
    _octomap_sub = node_handle.subscribe( "/octomap_full", 1, octomapCallback );

  

    /*  publish traj msgs to traj_server  */
    _traj_pub           = node_handle.advertise<quadrotor_msgs::PolynomialTrajectory>("trajectory", 10);
      
    /* timer */
    _planning_timer = node_handle.createTimer(ros::Duration(1.0/_planning_rate), planningCallBack);

    ros::Rate rate(100);

    bool status = ros::ok();
    while( status )
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }

}


 void timeMatrix(double current_time, int& index, Eigen::MatrixXd& polyder){
    double tseg = 0;
    double tcand;

    // find segment start time tseg
    for(int m = 0; m < _segment_num; m++){
        tcand = _Time[m];
        if(tcand < current_time){
            tseg = tcand;
            index = m;
        } else {
            break;
        }
    }
    tseg = current_time-tseg;
    polyder.resize(3, _param.n+1);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < _param.n+1; j++){
            if(i <= j)
                polyder(i, _param.n-j) = ((i==0)*1+(i==1)*j+(i==2)*j*(j-1)) * pow(tseg,j-i);
            else
                polyder(i, _param.n-j) = 0;
        }
    }
}


quadrotor_msgs::PolynomialTrajectory getBezierTraj()
{
    quadrotor_msgs::PolynomialTrajectory traj;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;
    traj.num_segment = _segment_num;

    int polyTotalNum = 0;
    // for(auto order:_poly_orderList)
    //     polyTotalNum += (order + 1);
    int maxOrder = 5;
    int poly_num1d = maxOrder + 1;
    polyTotalNum = _segment_num*(maxOrder+1);
    traj.coef_x.resize(polyTotalNum);
    traj.coef_y.resize(polyTotalNum);
    traj.coef_z.resize(polyTotalNum);

    int idx = 0;
    for(int i = 0; i < _segment_num; i++ )
    {    
        //int order = _poly_orderList[i];
        
        
        for(int j =0; j < poly_num1d; j++)
        { 
            traj.coef_x[idx] = _PolyCoeff(j +i*poly_num1d,        0);
            traj.coef_y[idx] = _PolyCoeff(j +i*poly_num1d,        1);
            traj.coef_z[idx] = _PolyCoeff(j +i*poly_num1d,        2);
            idx++;
        }
    }

    traj.header.frame_id = "/bernstein";
    traj.header.stamp = _start_time;

    traj.time.resize(_Time.size());
    traj.order.resize(_poly_orderList.size());

    traj.mag_coeff = 1.0;

    for (int idx = 0; idx < _Time.size(); ++idx){
        traj.time[idx] = _Time(idx);
    }
    for (int idx = 0; idx < _poly_orderList.size(); ++idx){
        traj.order[idx] = _poly_orderList[idx];
    }
    
    traj.start_yaw = 0.0;
    traj.final_yaw = 0.0;

    traj.trajectory_id = _traj_id;
    traj.action = quadrotor_msgs::PolynomialTrajectory::ACTION_ADD;

    return traj;
}


inline void getStateFromBezier(const MatrixXd & polyCoeff,  const double & t_now,VectorXd & ret)
{
    ret = VectorXd::Zero(3);

    if (t_now > _Time(_segment_num-1) ) {
        return;
    }

    int qn = 1;
    Eigen::MatrixXd pva;
    int index = 0;
    Eigen::MatrixXd polyder;
    timeMatrix(t_now, index, polyder);

        // polyder : (3,6) : ( pos,vel,acc)x( n+1 )
        // pva[qi] : (3,3) = (3,6)x(6,3)
        // coef[qi] : (6xM,3) = ( (n+1)xM, xyz)
        // coef[qi].block : (6,3) = ( n+1, xyz)
    pva = polyder * polyCoeff.block((_param.n + 1) * index, 0, (_param.n + 1), 3);  // size : 6x3 ,(n=5)

    ret = VectorXd::Zero(9);
    ret[0] = pva(0,0);
    ret[1] = pva(0,1);
    ret[2] = pva(0,2);
    ret[3] = pva(1,0);
    ret[4] = pva(1,1);
    ret[5] = pva(1,2);
    ret[6] = pva(2,0);
    ret[7] = pva(2,1);
    ret[8] = pva(2,2);

}

inline void getPosFromBezier(const MatrixXd & polyCoeff,  const double & t_now, Vector3d & ret)
{
    ret = VectorXd::Zero(3);

    // if (t_now > _Time(_segment_num-1) ) {
    //     return;
    // }

    int qn = 1;
    Eigen::MatrixXd pva;
    int index = 0;
    Eigen::MatrixXd polyder;
    timeMatrix(t_now, index, polyder);

        // polyder : (3,6) : ( pos,vel,acc)x( n+1 )
        // pva[qi] : (3,3) = (3,6)x(6,3)
        // coef[qi] : (6,3) = ( n+1, xyz)
        // coef[qi].block : (6,3) = ( n+1, xyz)
    pva = polyder * polyCoeff.block((_param.n + 1) * index, 0, (_param.n + 1), 3);  // size : 6x3 ,(n=5)
    ret(0) = pva(0,0);
    ret(1) = pva(0,1);
    ret(2) = pva(0,2);
}

// if return true, there is obstacle
bool checkSafeTrajectory(double check_time)
{   
    if(!_is_traj_exist) return false;

    traj_stop_pts_pcd.points.clear();

    double t_s = max(0.0, (_odom.header.stamp - _start_time).toSec());      
   

    for( double t = t_s ; t < _Time(_segment_num-1) ; t += 0.02 )
    {
        if(t > _stop_time) break; // _stop_time is check_time (max: 1, where _stop is 0.5sec)

        Vector3d traj_pt;
        getPosFromBezier(_PolyCoeff, t, traj_pt);
        
        PointXYZ pt;
        pt.x = traj_pt(0);
        pt.y = traj_pt(1);
        pt.z = traj_pt(2);
        traj_stop_pts_pcd.points.push_back(pt);
              
        float dist = distmap_obj.get()->getDistance(octomap::point3d(pt.x, pt.y, pt.z));
        float r = _mission.quad_size[0];
        
        if (dist < r + _param.grid_margin)
        {
          traj_stop_pts_pcd.width = traj_stop_pts_pcd.points.size();
          traj_stop_pts_pcd.height = 1;
          traj_stop_pts_pcd.is_dense = true;
          pcl::toROSMsg(traj_stop_pts_pcd, traj_stop_pts);
          traj_stop_pts.header.frame_id = "map";
          _vis_stop_traj_points.publish(traj_stop_pts);
          return true;
        }
    }

    traj_stop_pts_pcd.width = traj_stop_pts_pcd.points.size();
    traj_stop_pts_pcd.height = 1;
    traj_stop_pts_pcd.is_dense = true;
    pcl::toROSMsg(traj_stop_pts_pcd, traj_stop_pts);
    traj_stop_pts.header.frame_id = "map";
    _vis_stop_traj_points.publish(traj_stop_pts);

    return false;
}