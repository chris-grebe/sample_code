/**
 * @brief Takes in ground truth measurements from the simulator and publishes 
 * gps measurements with noise at a specified rate
 * @author Christopher Grebe <christopher.grebe@robotics.utias.utoronto.ca>
 */
#ifndef GPS_PUBLISHER
#define GPS_PUBLISHER

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <random>
#include <sstream>

class GPSPublisher{

  public:
    /**
     * @brief Constructor 
     * @param nh
     */
    explicit GPSPublisher(ros::NodeHandle nh);
    virtual ~GPSPublisher();

    /**
     * @brief publishes gps measurement 
     */ 
    void publishPoint();

  protected:
    /**
     * @brief callback for ground truth measurement 
     */
    void callback(const geometry_msgs::Quaternion& msg);

  private:
    std::atomic<float>m_x{};   // x position [m]
    std::atomic<float>m_z{};   // z position [m]

    ros::NodeHandle m_nh;       
    ros::Publisher m_pub;       
    ros::Subscriber m_sub;   

    std::default_random_engine m_generator_gps;
    std::normal_distribution<double> m_distribution_gps;  

    const static double STD_DEV; 

};

#endif /* GPS_PUBLISHER */