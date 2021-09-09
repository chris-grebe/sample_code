#include "simulator.h"

const double Simulator::STD_DEV = 5.0;

Simulator::Simulator(double cd, double rho, double mass, double gravity, double ac, double gt_rate, Eigen::Matrix<double, 4, 1> state_initial) : 
    m_cd            { cd },
    m_rho           { rho },
    m_m             { mass },
    m_g             { gravity },
    m_ac            { ac },
    m_gt_rate       { gt_rate },
    m_state_initial { state_initial }
    
{}
Simulator::~Simulator() 
{}

void Simulator::updateState(Eigen::Matrix<double, 4, 1>& state, Eigen::Matrix<double, 4, 1>& true_meas)
{
  // previous state 
  const double x = state(0,0);          // x position [m]
  const double z = state(1,0);          // z position [m]
  const double vx = state(2,0);         // x acceleration [m/s]
  const double vz = state(3,0);         // z acceleration [m/s]
  const double dt = 1.0/m_gt_rate;      // update period [s]

  // set simulator wind according to spec. wind = -5m/s at height < 1000m and 5m/s at height > 1000m
  double v_wind = 0;
  if (z > 1000)
  {
      v_wind = 5.0;
  } else 
  {
      v_wind = -5.0;
  }

  // compute forces acting on balloon 
  double f_drag_x = m_cd*0.5*m_rho*std::pow(vx - v_wind, 2)*m_ac; // drag force in x
  if (v_wind- vx < 0)
  {
    f_drag_x = -f_drag_x;
  }
  const double f_drag_z = m_cd*0.5*m_rho*std::pow(vz, 2)*m_ac;    // drag force in z
  const double f_lift_z = 1500.0;                                 // lifing force in z
  const double f_grav_z = m_m*m_g;                                // force of gravity in z

  // x and z acceleration 
  const double vx_dot = f_drag_x/m_m;                                                  
  const double vz_dot = (f_lift_z - f_drag_z - f_grav_z)/m_m;

  // position process noise
  const double noise_pos_x = m_distribution(m_generator)*STD_DEV;
  const double noise_pos_z = m_distribution(m_generator)*STD_DEV;

  // state update
  state(0,0) = x + dt*vx + noise_pos_x;
  state(1,0) = z + dt*vz + noise_pos_z;
  state(2,0) = vx + dt*vx_dot;
  state(3,0) = vz + dt*vz_dot;

  // true measurements
  true_meas(0,0) = x;       // gps x
  true_meas(1,0) = z;       // gps z
  true_meas(2,0) = vx_dot;  // acceleration x
  true_meas(3,0) = vz_dot;  // acceleration z
}

void Simulator::publishGroundTruth(ros::NodeHandle n)
{
  ros::Publisher gt_meas_pub = n.advertise< geometry_msgs::Quaternion >("gt_meas", 1000);
  ros::Rate loop_rate(m_gt_rate); 

  Eigen::Matrix<double, 4, 1> state = m_state_initial; 
  Eigen::Matrix<double, 4, 1> true_meas;

  // loop publishes gt measurements at m_gt_rate
  while (ros::ok())
  {
    updateState(state, true_meas);

    ROS_INFO("TURE_STATE x=%lf z=%lf vx=%lf vz=%lf", state(0,0), state(1,0), state(2,0), state(3,0 ));
    ROS_INFO("TRUE_MEAS x=%lf z=%lf vx=%lf vz=%lf", true_meas(0,0), true_meas(1,0), true_meas(2,0), true_meas(3,0 ));

    geometry_msgs::Quaternion quat;   // using quaternion for this example since it has 4 elements - in practice would create special message type

    quat.x = true_meas(0,0);
    quat.y = true_meas(1,0);
    quat.z = true_meas(2,0);
    quat.w = true_meas(3,0);

    gt_meas_pub.publish(quat);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

int main(int argc, char **argv)
{
  ROS_INFO("Starting Simulation");
  ros::init(argc, argv, "simulator");
  ros::NodeHandle n;

  Eigen::Matrix<double, 4, 1> state_initial = Eigen::MatrixXd::Zero(4,1); // set initial state to zero position and zero velocity

  auto gen = std::make_unique<Simulator>(/*cd = */ 0.35, /* rho =*/ 1.25, /* mass = */ 100,  /*gravity = */ 9.81, /* ac = */ 5, /* gt rate = */ 400, state_initial);
  gen->publishGroundTruth(n);

  return 0;
}
