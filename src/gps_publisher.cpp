#include "gps_publisher.h"

const double GPSPublisher::STD_DEV = 60.0;

GPSPublisher::GPSPublisher(ros::NodeHandle nh):
  m_nh        { nh },
  m_sub       { m_nh.subscribe("/gt_meas", 1000, &GPSPublisher::callback, this) },
  m_pub       { m_nh.advertise<geometry_msgs::Point>("/gps", 1000) }
{}
GPSPublisher::~GPSPublisher() 
{}

void GPSPublisher::callback(const geometry_msgs::Quaternion& msg)
{
  m_x.store(msg.x);   // ground truth x position [m]
  m_z.store(msg.y);   // ground truth z position [m]
}

void GPSPublisher::publishPoint()
{
  const double noise_pos_x = m_distribution_gps(m_generator_gps)*STD_DEV;
  const double noise_pos_z = m_distribution_gps(m_generator_gps)*STD_DEV;
  geometry_msgs::Point point;
  point.x = m_x.load() + noise_pos_x; 
  point.z = m_z.load() + noise_pos_z;
  m_pub.publish(point);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_publisher");
  ros::NodeHandle n;
  auto gps = std::make_unique<GPSPublisher>(n);

  ros::Rate loop_rate(1); // gps publishes measurements at frequency of 1 Hz
 
  // publishes gps measurements at specified rate
  while (ros::ok())
  {
    gps->publishPoint();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}