#include "estimator.h"

const double Estimator::GRAVITY = 9.81;
const double Estimator::UPDATE_RATE = 10; // Estimator update rate [Hz]

Estimator::Estimator( ros::NodeHandle n) : 
    m_sub_acc     { n.subscribe( "/accel", 1000, &Estimator::accelCallback, this) },
    m_sub_gps     { n.subscribe( "/gps", 1000, &Estimator::gpsCallback, this) },
    m_pub         { n.advertise< geometry_msgs::Quaternion >( "/estimator", 1000) },
    m_matrix_R    { Eigen::MatrixXd::Identity(2,2)*60 } //TODO: Need to set
{
  m_matrix_Q <<     5.0,    0,      0,      0.0,
                    0,      5.0,    0,      0.0,
                    0,      0,      0.981,  0.0,
                    0.0,    0.0,    0.0,    0.981;
}
Estimator::~Estimator()
{}

void Estimator::accelCallback(const geometry_msgs::Accel::ConstPtr& msg)
{
  // propogate to current time with old accel
  double dt = ros::Time::now().toSec() - m_estimator_time;
  propogateState(dt);

  // update with new acceleration
  m_accel_x = msg->linear.x;
  m_accel_z = msg->linear.z - GRAVITY;

  m_estimator_time += dt;
}

void Estimator::gpsCallback(const geometry_msgs::Point::ConstPtr& msg)
{
  double dt = ros::Time::now().toSec() - m_estimator_time;
  propogateState(dt);

  Eigen::Matrix<double,2,1> meas;
  meas(0,0) = msg->x;
  meas(1,0) = msg->z;

  correctState(dt, meas);

  m_estimator_time += dt;
}

Eigen::Matrix<double, 2, 1> Estimator::measurementModel(const Eigen::Matrix<double, 4, 1>& state)
{
    Eigen::Matrix<double, 2, 1> meas;
    meas(0,0) = m_state_current(0,0);
    meas(1,0) = m_state_current(1,0);
    return meas;
}
        
Eigen::Matrix<double, 4, 1> Estimator::motionModel(const Eigen::Matrix<double, 4, 1>& state, double dt)
{
  Eigen::Matrix<double, 4, 1> next_state;

  next_state(0,0) = state(0,0) + dt*state(2,0) + std::pow(dt,2)*0.5*m_accel_x; 
  next_state(1,0) = state(1,0) + dt*state(3,0) + std::pow(dt,2)*0.5*m_accel_z;
  next_state(2,0) = state(2,0) + dt*m_accel_x;
  next_state(3,0) = state(3,0) + dt*m_accel_z;

  return next_state;
}

Eigen::Matrix<double, 4, 4> Estimator::computeF(double dt)
{
  Eigen::Matrix<double, 4, 4> mat = Eigen::MatrixXd::Identity(4,4);
  mat(0,2) = dt;
  mat(1,3) = dt;

  return mat;
}

Eigen::Matrix<double, 2, 4> Estimator::computeG()
{
  Eigen::Matrix<double, 2, 4> mat = Eigen::MatrixXd::Zero(2,4);
  mat(0,0) = 1.0;
  mat(1,1) = 1.0;

  return mat;
}

Eigen::Matrix<double, 4, 4> Estimator::computeQ(double dt)
{
  Eigen::Matrix<double, 4, 4> S = Eigen::MatrixXd::Zero(4,4);
  S(0,0) = std::pow(dt, 3)/3;
  S(1,1) = std::pow(dt, 3)/3;
  S(2,2) = dt;
  S(3,3) = dt;
  S(0,2) = std::pow(dt, 2)/2;
  S(1,3) = std::pow(dt, 2)/2;
  S(2,0) = std::pow(dt, 2)/2;
  S(3,1) = std::pow(dt, 2)/2;
  
  //S(2,2) = dt;
  //S(3,3) = dt;
  return S;//*m_matrix_Q*S.transpose();
}

Eigen::Matrix<double, 2, 2> Estimator::computeR()
{
  return m_matrix_R;
}

void Estimator::propogateState(double dt)
{
  m_state_current = motionModel(m_state_current, dt);
  
  const Eigen::Matrix<double, 4, 4> F = computeF(dt);
  
  m_covariance_current = F*m_covariance_current*F.transpose() + computeQ(dt);
}

void Estimator::propogateStateDisp()
{
  const double dt = ros::Time::now().toSec() - m_estimator_time;
  Eigen::Matrix<double, 4, 1> state_disp = motionModel(m_state_current, dt);
   // propogate to current time with old accel
    
      geometry_msgs::Quaternion quat;

      quat.x = state_disp(0,0);
      quat.y = state_disp(1,0);
      quat.z = state_disp(2,0);
      quat.w = state_disp(3,0);

  m_pub.publish( quat );
}

void Estimator::correctState(double dt, const Eigen::Matrix<double,2,1>& meas)
{
  const auto R = computeR();
  const auto G = computeG();

  const auto temp = G*m_covariance_current*G.transpose() + R;

  const auto K = m_covariance_current*G.transpose()*temp.inverse();
  m_covariance_current = (Eigen::MatrixXd::Identity(4,4)- K*G)*m_covariance_current;
  m_state_current = m_state_current + K*(meas - measurementModel(m_state_current));

}

int main(int argc, char** argv)
{
    ROS_INFO("Start");
    ros::init(argc, argv, "estimator");
    ros::NodeHandle n;

    auto gen = std::make_unique<Estimator>(n);

    ros::Rate loop_rate(Estimator::UPDATE_RATE);

    while (ros::ok())
    {
      gen->propogateStateDisp();
      ros::spinOnce();
      loop_rate.sleep();
    }
    
    return 0;
}


