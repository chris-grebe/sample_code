#include "accel_publisher.h"

const double AccelPublisher::GRAVITY = 9.81;
const double AccelPublisher::STD_DEV = 0.981;

AccelPublisher::AccelPublisher(ros::NodeHandle nh) :
  m_nh        { nh },
  m_sub       { m_nh.subscribe("/gt_meas", 1000, &AccelPublisher::callback, this) },
  m_pub       { m_nh.advertise<geometry_msgs::Accel>("/accel", 1000) } 
{}
AccelPublisher::~AccelPublisher() 
{}

void AccelPublisher::callback(const geometry_msgs::Quaternion& msg)
{
  m_x.store(msg.z); // ground truth x acceleration [m/s^2]
  m_z.store(msg.w); // ground truth z acceleration [m/s^2]
}

void AccelPublisher::publishAccel()
{
  const double noise_accel_x = m_distribution(m_generator)*STD_DEV;
  const double noise_accel_z = m_distribution(m_generator)*STD_DEV;
  geometry_msgs::Accel acc; 
  acc.linear.x = m_x.load() + noise_accel_x;
  acc.linear.z = m_z.load() + GRAVITY + noise_accel_z;
  m_pub.publish(acc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "accel_publisher");
  ros::NodeHandle n;
  auto acc = std::make_unique<AccelPublisher>(n);

  ros::Rate loop_rate(40); // accelerometer publishes measurements at frequency of 40 Hz
 
  // publishes gps measurements at specified rate
  while (ros::ok())
  {
    acc->publishAccel();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}