/**
 * @brief Takes in ground truth measurements from the simulator and publishes 
 * accelerometer measurements with noise at a specified rate
 * @author Christopher Grebe <christopher.grebe@robotics.utias.utoronto.ca>
 */
#ifndef ACCEL_PUBLISHER
#define ACCEL_PUBLISHER

#include "ros/ros.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Quaternion.h"
#include <random>
#include <sstream>

class AccelPublisher{

  public:
  /**
    * @brief constructor
    */ 
    explicit AccelPublisher(ros::NodeHandle nh);
    virtual ~AccelPublisher();

  /**
    * @brief publishes accelerometer measurement 
    */ 
    void publishAccel();

  protected:
  /**
    * @brief callback for groundtruth measurement
    * @param msg ground truth measurement message
    */ 
    void callback(const geometry_msgs::Quaternion& msg);

  private:
    std::atomic<float>m_x{};   // x acceleration [m/s^2]
    std::atomic<float>m_z{};   // z acceleration [m/s^2]

    ros::NodeHandle m_nh;       
    ros::Publisher m_pub;       
    ros::Subscriber m_sub;   

    const static double GRAVITY;  
    const static double STD_DEV;     

    std::default_random_engine m_generator;
    std::normal_distribution<double> m_distribution;

};

#endif /* ACCEL_PUBLISHER */