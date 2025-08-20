#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>

class TeleopOrchestrator
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Subscriber pose_cmd_sub_;
    ros::Subscriber gait_sub_;
    ros::Publisher robot_status_pub_;
    ros::Publisher imu_pub_;
    ros::Timer cmd_vel_timeout_;
    ros::Timer publish_timer_;
    
    geometry_msgs::Twist last_cmd_vel_;
    bool cmd_vel_active_;
    
    static constexpr double CMD_VEL_TIMEOUT = 0.25;
    static constexpr double PUBLISH_RATE = 20.0;

public:
    TeleopOrchestrator() : cmd_vel_active_(false)
    {
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &TeleopOrchestrator::cmdVelCallback, this);
        pose_cmd_sub_ = nh_.subscribe("/pose_cmd", 1, &TeleopOrchestrator::poseCmdCallback, this);
        gait_sub_ = nh_.subscribe("/set_gait", 1, &TeleopOrchestrator::gaitCallback, this);
        
        robot_status_pub_ = nh_.advertise<sensor_msgs::BatteryState>("/robot_status", 1);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data", 1);
        
        publish_timer_ = nh_.createTimer(ros::Duration(1.0/PUBLISH_RATE), &TeleopOrchestrator::publishTelemetry, this);
        
        ROS_INFO("Teleop Orchestrator iniciado");
    }
    
private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        last_cmd_vel_ = *msg;
        cmd_vel_active_ = true;
        
        cmd_vel_timeout_.stop();
        cmd_vel_timeout_ = nh_.createTimer(ros::Duration(CMD_VEL_TIMEOUT), &TeleopOrchestrator::cmdVelTimeoutCallback, this, true);
        
        ROS_INFO("Comando recibido: linear.x=%.2f, angular.z=%.2f", msg->linear.x, msg->angular.z);
    }
    
    void poseCmdCallback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        ROS_INFO("Comando de pose recibido");
    }
    
    void gaitCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Gait cambiado a: %s", msg->data.c_str());
    }
    
    void cmdVelTimeoutCallback(const ros::TimerEvent&)
    {
        cmd_vel_active_ = false;
        ROS_WARN("Timeout cmd_vel - Robot detenido por seguridad");
    }
    
    void publishTelemetry(const ros::TimerEvent&)
    {
        // Publicar batería simulada
        sensor_msgs::BatteryState battery_msg;
        battery_msg.header.stamp = ros::Time::now();
        battery_msg.voltage = 12.6;
        battery_msg.percentage = 0.75;
        robot_status_pub_.publish(battery_msg);
        
        // Publicar IMU simulada
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "base_link";
        imu_msg.orientation.w = 1.0;
        imu_pub_.publish(imu_msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_orchestrator");
    TeleopOrchestrator orchestrator;
    ros::spin();
    return 0;
}
