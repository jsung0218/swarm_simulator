#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include "bezier_base.h"

const int  _DIM_x = 0;
const int  _DIM_y = 1;
const int  _DIM_z = 2;

using namespace std;

int _poly_order_min, _poly_order_max;
typedef enum ServerState{INIT, TRAJ, HOVER} e_ServerState;
class TrajectoryServer
{
private:

    // Subscribers
    ros::Subscriber _odom_sub;
    ros::Subscriber _traj_sub;

    // publishers
    ros::Publisher _cmd_pub;
    ros::Publisher _vis_cmd_pub;
    ros::Publisher _vis_vel_pub;
    ros::Publisher _vis_acc_pub;
    ros::Publisher _vis_traj_pub;
    ros::Publisher _vis_traj_points;
    
    // configuration for trajectory
    int _n_segment = 0;
    int _traj_id = 0;
    uint32_t _traj_flag = 0;
    Eigen::VectorXd _Time;
    MatrixXd _PolyCoeff;
    vector<int> _order;
    
    double _vis_traj_width = 0.2;
    double mag_coeff;
    ros::Time _final_time = ros::TIME_MIN;
    ros::Time _start_time = ros::TIME_MAX;
    double _start_yaw = 0.0, _final_yaw = 0.0;

    // state of the server
    e_ServerState state = INIT;
    e_ServerState state_prev = INIT;;
    
    nav_msgs::Odometry _odom;
    quadrotor_msgs::PositionCommand _cmd;
    geometry_msgs::PoseStamped _vis_cmd;

    visualization_msgs::Marker _vis_vel, _vis_acc;
    visualization_msgs::Marker _vis_traj;

    sensor_msgs::PointCloud2 traj_pts;
    pcl::PointCloud<pcl::PointXYZ> traj_pts_pcd;
public:
    
    vector<Eigen::VectorXd> CList;   // Position coefficients vector, used to record all the pre-compute 'n choose k' combinatorial for the bernstein coefficients .
    vector<Eigen::VectorXd> CvList; // Velocity coefficients vector.
    vector<Eigen::VectorXd> CaList; // Acceleration coefficients vector.

    TrajectoryServer(ros::NodeHandle & handle)
    {   
        _odom_sub = 
            handle.subscribe("odometry", 50, &TrajectoryServer::rcvOdometryCallback, this);

        _traj_sub =
            handle.subscribe("trajectory", 2, &TrajectoryServer::rcvTrajectoryCallabck, this);

        _cmd_pub = 
            handle.advertise<quadrotor_msgs::PositionCommand>("position_command", 50);

        _vis_cmd_pub = 
            handle.advertise<geometry_msgs::PoseStamped>("desired_position", 50);

        _vis_vel_pub = 
            handle.advertise<visualization_msgs::Marker>("desired_velocity", 50);
        
        _vis_acc_pub = 
            handle.advertise<visualization_msgs::Marker>("desired_acceleration", 50);

        _vis_traj_pub = 
            handle.advertise<visualization_msgs::Marker>("trajectory_vis", 1);

        _vis_traj_points = 
            handle.advertise<sensor_msgs::PointCloud2>("trajectory_vis_points", 1);
        
        double pos_gain[3] = {5.7, 5.7, 6.2};
        double vel_gain[3] = {3.4, 3.4, 4.0};
        setGains(pos_gain, vel_gain);

        _vis_traj.header.stamp       = ros::Time::now();
        _vis_traj.header.frame_id    = "map";

        _vis_traj.ns = "trajectory/trajectory";
        _vis_traj.id = 0;
        _vis_traj.type = visualization_msgs::Marker::SPHERE_LIST;
        _vis_traj.action = visualization_msgs::Marker::ADD;
        _vis_traj.scale.x = _vis_traj_width;
        _vis_traj.scale.y = _vis_traj_width;
        _vis_traj.scale.z = _vis_traj_width;
        _vis_traj.pose.orientation.x = 0.0;
        _vis_traj.pose.orientation.y = 0.0;
        _vis_traj.pose.orientation.z = 0.0;
        _vis_traj.pose.orientation.w = 1.0;
        _vis_traj.color.r = 0.0;
        _vis_traj.color.g = 0.0;
        _vis_traj.color.b = 1.0;
        _vis_traj.color.a = 0.3;
        _vis_traj.points.clear();
    }

    // not used
    void setGains(double pos_gain[3], double vel_gain[3])
    {
        _cmd.kx[_DIM_x] = pos_gain[_DIM_x];
        _cmd.kx[_DIM_y] = pos_gain[_DIM_y];
        _cmd.kx[_DIM_z] = pos_gain[_DIM_z];

        _cmd.kv[_DIM_x] = vel_gain[_DIM_x];
        _cmd.kv[_DIM_y] = vel_gain[_DIM_y];
        _cmd.kv[_DIM_z] = vel_gain[_DIM_z];
    }

    bool cmd_flag = false;
    // after recieving odem, position cmd is published according to state except ACTION
    void rcvOdometryCallback(const nav_msgs::Odometry & odom)
    {
        if (odom.child_frame_id == "X" || odom.child_frame_id == "O") return ;
        // #1. store the odometry
        _odom = odom;
        _vis_cmd.header = _odom.header;
        _vis_cmd.header.frame_id = "/map";
        

        if(state == INIT && fabs(_odom.pose.pose.position.z  - 1.0) < 0.1 )
            cmd_flag = true;

        if(state == INIT )
        {
            //ROS_WARN("[TRAJ SERVER] Pub initial pos command");
            _cmd.position   = _odom.pose.pose.position;
            
            // if(!cmd_flag)
            //     _cmd.position.z =  1.5;
            // else
            //     _cmd.position.z =  1.5;
            
            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "/map";
            _cmd.trajectory_flag = _traj_flag;

            _cmd.velocity.x = 0.0;
            _cmd.velocity.y = 0.0;
            _cmd.velocity.z = 0.0;
            
            _cmd.acceleration.x = 0.0;
            _cmd.acceleration.y = 0.0;
            _cmd.acceleration.z = 0.0;
            _cmd_pub.publish(_cmd);

            _vis_cmd.pose.position.x = _cmd.position.x;
            _vis_cmd.pose.position.y = _cmd.position.y;
            _vis_cmd.pose.position.z = _cmd.position.z;
            _vis_cmd_pub.publish(_vis_cmd);

            return;
        }
        // #2. try to publish command
        // ROS_WARN("[ODOM] Enter just before pub function");
        
        pubPositionCommand();

        // #3. try to calculate the new state
        if (state == TRAJ && ( (odom.header.stamp - _start_time).toSec() / mag_coeff > (_final_time - _start_time).toSec() ) )
        {
            // ROS_WARN("[ODOM] Time is exceed : %1.2f", (odom.header.stamp - _start_time).toSec() );
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
    }



 void timeMatrix(double current_time, int& index, Eigen::MatrixXd& polyder){
    double tseg = 0;
    double tcand;

    // find segment start time tseg
    for(int m = 0; m < _n_segment; m++){
        tcand = _Time[m];
        if(tcand < current_time){
            tseg = tcand;
            index = m;
        } else {
            break;
        }
    }
    tseg = current_time-tseg;
    int MaxOrder = 5;
    polyder.resize(3, MaxOrder+1);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < MaxOrder+1; j++){
            if(i <= j)
                polyder(i, MaxOrder-j) = ((i==0)*1+(i==1)*j+(i==2)*j*(j-1)) * pow(tseg,j-i);
            else
                polyder(i, MaxOrder-j) = 0;
        }
    }
}

    void rcvTrajectoryCallabck(const quadrotor_msgs::PolynomialTrajectory & traj)
    {
        //ROS_WARN("[SERVER] Recevied The Trajectory with %.3lf.", _start_time.toSec());
        //ROS_WARN("[SERVER] Now the odom time is : ");
        // #1. try to execuse the action
        
        if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ADD)
        {   
            ROS_WARN("[SERVER] Loading the trajectory.");
            if ((int)traj.trajectory_id < _traj_id) return ;


            state = TRAJ;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
            _traj_id = traj.trajectory_id;
            _n_segment = traj.num_segment;
            // ROS_WARN("[SERVER] seg : %d, id : %d, ", _n_segment, _traj_id );
            if (_n_segment == 0 )
            {
                ROS_WARN("[SERVER] zero segment.");
                return;
            }

            _final_time = _start_time = traj.header.stamp;
            _Time.resize(_n_segment+1);

            _order.clear();
            int idx;
            for (idx = 0; idx < _n_segment; ++idx)
            {
                _Time(idx) = traj.time[idx];
                _order.push_back(traj.order[idx]);
            }
            _Time(idx) = traj.time[idx];
            for (idx = 0; idx < _n_segment;++idx)
            {
                _final_time += ros::Duration(_Time(idx+1)-_Time(idx));
            }
            // ROS_WARN("[SERVER] Time is assigned");

            _start_yaw = traj.start_yaw;
            _final_yaw = traj.final_yaw;
            mag_coeff  = traj.mag_coeff;

            int max_order = 5; 
            int poly_num1d = max_order+1;
            
            _PolyCoeff = MatrixXd::Zero( poly_num1d*_n_segment, 3);
            // ROS_WARN("[SERVER] _PolyCoeff is zerorized ");
            
            ROS_WARN("stack the coefficients");
            idx = 0;
            for (int i = 0; i < _n_segment; ++i)
            {     
                //int order = traj.order[i];
                
                for (int j = 0; j < poly_num1d; ++j)
                {
                    _PolyCoeff(j +i*poly_num1d,        0) = traj.coef_x[idx];
                    _PolyCoeff(j +i*poly_num1d,        1) = traj.coef_y[idx];
                    _PolyCoeff(j +i*poly_num1d,        2) = traj.coef_z[idx];
                    idx++;
                }
            }
            // ROS_WARN("coeff is assigned : %d", idx);
            #if 0
            #endif
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_ABORT) 
        {
            ROS_WARN("[SERVER] Aborting the trajectory.");
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED;
        }
        else if (traj.action == quadrotor_msgs::PolynomialTrajectory::ACTION_WARN_IMPOSSIBLE)
        {
            state = HOVER;
            _traj_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_IMPOSSIBLE;
        }
        if (state_prev !=state)
        {
            // ROS_WARN(" State is changed from %d to %d", state_prev, state);
            // for (int idx = 0; idx < _n_segment; ++idx)
            // {
            //     ROS_WARN(" [%d] : %1.2f" , idx, _Time(idx) );
            // }
        }
        state_prev = state;
        // ROS_WARN("[subTraj] exit function %d", state);
    }

    void pubPositionCommand()
    {
        // #1. check if it is right state
        static int gCount = 0;
        static double first_time = ros::Time::now().toSec();
        if (state == INIT) return;
        if (state == HOVER)
        {
            if (_cmd.header.frame_id != "/map"){
                _cmd.position = _odom.pose.pose.position;
            }

            _cmd.header.stamp = _odom.header.stamp;
            _cmd.header.frame_id = "/map";
            _cmd.trajectory_flag = _traj_flag;

            _cmd.velocity.x = 0.0;
            _cmd.velocity.y = 0.0;
            _cmd.velocity.z = 0.0;
            
            _cmd.acceleration.x = 0.0;
            _cmd.acceleration.y = 0.0;
            _cmd.acceleration.z = 0.0;
        }
        // #2. locate the trajectory segment
        if (state == TRAJ)
        {
            // ROS_WARN("[SERVER] Enter Pub of Pos. TRAJ");
            _cmd.header.stamp = _odom.header.stamp;

            _cmd.header.frame_id = "/map";
            _cmd.trajectory_flag = _traj_flag;
            _cmd.trajectory_id = _traj_id;

            double t = max(0.0, (_odom.header.stamp - _start_time).toSec());// / mag_coeff;

            // ROS_WARN("[SERVER] Curretn Time : %1.2f sec", t);
            //cout<<"t: "<<t<<endl; 
            _cmd.yaw_dot = 0.0;
            _cmd.yaw = _start_yaw + (_final_yaw - _start_yaw) * t 
                / ((_final_time - _start_time).toSec() + 1e-9);

            // #3. calculate the desired states
             Eigen::MatrixXd pva;
            int index = 0;
            Eigen::MatrixXd polyder;
            timeMatrix(t, index, polyder);
            int maxOrder = 5;

                // polyder : (3,6) : ( pos,vel,acc)x( n+1 )
                // pva[qi] : (3,3) = (3,6)x(6,3)
                // coef[qi] : (6xM,3) = ( (n+1)xM, xyz)
                // coef[qi].block : (6,3) = ( n+1, xyz)
            pva = polyder * _PolyCoeff.block((maxOrder + 1) * index, 0, (maxOrder+ 1), 3);  // size : 6x3 ,(n=5)

            _cmd.position.x = pva(0,0);
            _cmd.position.y = pva(0,1);
            _cmd.position.z = pva(0,2);
            _cmd.velocity.x = pva(1,0);
            _cmd.velocity.y = pva(1,1);
            _cmd.velocity.z = pva(1,2);
            _cmd.acceleration.x =  pva(2,0);
            _cmd.acceleration.y =  pva(2,1);
            _cmd.acceleration.z =  pva(2,2);

            //ROS_WARN("[SERVER] the time : %.3lf\n, n = %d, m = %d", t, _n_order, _n_segment);

        }
        // #4. just publish
       
        // if (gCount++ % 1000 == 0)
        // {
        //     double cur_time = ros::Time::now().toSec() - first_time;
        //     ROS_WARN(" [My] (%1.2f s) Position cmd : (%1.2f,%1.2f,%1.2f)", cur_time, _cmd.position.x, _cmd.position.y, _cmd.position.z ) ;
        //     for (int i = 0; i < _n_segment; ++i)
        //     {
        //         ROS_WARN(" [%d] : %1.2f coef : %1.2f,%1.2f,%1.2f,%1.2f,%1.2f,%1.2f" , i, _Time(i),
        //         _PolyCoeff( 0+6*i, 0),
        //         _PolyCoeff( 1+6*i, 0),
        //         _PolyCoeff( 2+6*i, 0),
        //         _PolyCoeff( 3+6*i, 0),
        //         _PolyCoeff( 4+6*i, 0),
        //         _PolyCoeff( 5+6*i, 0) );
        //     }
        // }
        // ROS_WARN("[SERVER] Before of publishing cmd");
        _cmd_pub.publish(_cmd);

        _vis_cmd.header = _cmd.header;
        _vis_cmd.pose.position.x = _cmd.position.x;
        _vis_cmd.pose.position.y = _cmd.position.y;
        _vis_cmd.pose.position.z = _cmd.position.z;
        
        tf::Quaternion q_ = tf::createQuaternionFromYaw(_cmd.yaw);
        geometry_msgs::Quaternion odom_quat;
        tf::quaternionTFToMsg(q_, odom_quat);
        _vis_cmd.pose.orientation = odom_quat;
        _vis_cmd_pub.publish(_vis_cmd);
        
        _vis_vel.ns = "vel";
        _vis_vel.id = 0;
        _vis_vel.header.frame_id = "/map";
        _vis_vel.type = visualization_msgs::Marker::ARROW;
        _vis_vel.action = visualization_msgs::Marker::ADD;
        _vis_vel.color.a = 1.0;
        _vis_vel.color.r = 0.0;
        _vis_vel.color.g = 1.0;
        _vis_vel.color.b = 0.0;

        _vis_vel.header.stamp = _odom.header.stamp;
        _vis_vel.points.clear();

        geometry_msgs::Point pt;
        pt.x = _cmd.position.x;
        pt.y = _cmd.position.y;
        pt.z = _cmd.position.z;
        
        _vis_traj.points.push_back(pt);
        _vis_traj_pub.publish(_vis_traj);

        pcl::PointXYZ point(pt.x, pt.y, pt.z);
        traj_pts_pcd.points.push_back(point);
        traj_pts_pcd.width = traj_pts_pcd.points.size();
        traj_pts_pcd.height = 1;
        traj_pts_pcd.is_dense = true;
         
        pcl::toROSMsg(traj_pts_pcd, traj_pts);
        traj_pts.header.frame_id = "map";
        _vis_traj_points.publish(traj_pts);

        _vis_vel.points.push_back(pt);
        
        pt.x = _cmd.position.x + _cmd.velocity.x;
        pt.y = _cmd.position.y + _cmd.velocity.y;
        pt.z = _cmd.position.z + _cmd.velocity.z;
        
        _vis_vel.points.push_back(pt);

        _vis_vel.scale.x = 0.2;
        _vis_vel.scale.y = 0.4;
        _vis_vel.scale.z = 0.4;

        _vis_vel_pub.publish(_vis_vel);

        _vis_acc.ns = "acc";
        _vis_acc.id = 0;
        _vis_acc.header.frame_id = "/map";
        _vis_acc.type = visualization_msgs::Marker::ARROW;
        _vis_acc.action = visualization_msgs::Marker::ADD;
        _vis_acc.color.a = 1.0;
        _vis_acc.color.r = 1.0;
        _vis_acc.color.g = 1.0;
        _vis_acc.color.b = 0.0;

        _vis_acc.header.stamp = _odom.header.stamp;

        _vis_acc.points.clear();
        pt.x = _cmd.position.x;
        pt.y = _cmd.position.y;
        pt.z = _cmd.position.z;

        _vis_acc.points.push_back(pt);
        
        pt.x = _cmd.position.x + _cmd.acceleration.x;
        pt.y = _cmd.position.y + _cmd.acceleration.y;
        pt.z = _cmd.position.z + _cmd.acceleration.z;

        _vis_acc.points.push_back(pt);

        _vis_acc.scale.x = 0.2;
        _vis_acc.scale.y = 0.4;
        _vis_acc.scale.z = 0.4;

        _vis_acc_pub.publish(_vis_acc);
    }
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "gradient_trajectory_server_node");
    ros::NodeHandle handle("~");

    handle.param("optimization/poly_order_min", _poly_order_min,  5);
    handle.param("optimization/poly_order_max", _poly_order_max,  10);
    TrajectoryServer server(handle);

    Bernstein _bernstein;
    if(_bernstein.setParam(_poly_order_min, _poly_order_max, 3) == -1) // Here default the _minimize_order, give it 3, no use in the node
    {
        ROS_ERROR(" The trajectory order is set beyond the library's scope, please re-set ");
    }

    server.CList  = _bernstein.getC();
    server.CvList = _bernstein.getC_v();
    server.CaList = _bernstein.getC_a();

    ros::spin();

    return 0;
}