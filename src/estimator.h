/**
 * @brief esimation code
 * @author Christopher Grebe <christopher.grebe@robotics.utias.utoronto.ca>
 */
#ifndef ESTIMATOR
#define ESTIMATOR

#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Point.h"
#include <Eigen/Dense>
#include <math.h>

class Estimator 
{

    public:
        /**
         * @brief Constructor
         * @param n
         */ 
        explicit Estimator(ros::NodeHandle n);
        virtual ~Estimator();

        /**
         * @brief propogate state forward in time
         * @param dt time to propogate state forward [s]
         * @return next state [x position [m]; z position [m]; x acceleration [m/s^2]; z acceleration [m/s^2]]
         */
        void propogateState(double dt);

        /**
         * @brief publishes state without influencing core estimator
         */
        void propogateStateDisp();

        /**
         * @brief correct current state estimate using gps measurement
         * @param dt time to propogate state forward [s] as state must be propogate to time of gps measurement 
         * @param meas [x position [m]; z position [m]]
         */
        void correctState(double dt, const Eigen::Matrix<double,2,1>& meas);

        const static double UPDATE_RATE; // Estimator update rate [Hz]

    protected:
        /**
         * @brief callback for acceleration measurement
         * @param msg acceleration measurement 
         */ 
        void accelCallback(const geometry_msgs::Accel::ConstPtr& msg);
        
        /**
         * @brief callback for gps measurement
         * @param msg gps measurement 
         */ 
        void gpsCallback(const geometry_msgs::Point::ConstPtr& msg);

        /**
         * @brief noise free measurement model
         * @param state current state [x position [m]; z position [m]; x acceleration [m/s^2]; z acceleration [m/s^2]]
         * @return exteroceptive measurement [x position [m]; z position [m]]
         */
        Eigen::Matrix<double, 2, 1> measurementModel(const Eigen::Matrix<double, 4, 1>& state);
        
        /**
         * @brief noise free motion model
         * @param state [x position [m]; z position [m]; x acceleration [m/s^2]; z acceleration [m/s^2]]
         * @param dt time to propogate state forward
         * @return next state [x position [m]; z position [m]; x acceleration [m/s^2]; z acceleration [m/s^2]]
         */
        Eigen::Matrix<double, 4, 1> motionModel(const Eigen::Matrix<double, 4, 1>& state, double dt);

        /**
         * @brief motion model jacobian
         * @param dt change in time [s]
         */
        Eigen::Matrix<double, 4, 4> computeF(double dt);

        /**
         * @brief measurement model jacobian
         */
        Eigen::Matrix<double, 2, 4> computeG();

        /**
         * @brief 
         * @param dt change in time [s]
         */
        Eigen::Matrix<double, 4, 4> computeQ(double dt);

        /**
         * @brief
         */
        Eigen::Matrix<double, 2, 2> computeR();
    
    private:
        Eigen::Matrix<double, 4, 1> m_state_current;        // current state estimte
        Eigen::Matrix<double, 4, 4> m_covariance_current;   // current state covariance
        
        ros::Subscriber m_sub_acc;
        ros::Subscriber m_sub_gps;
        ros::Publisher m_pub;

        Eigen::Matrix<double, 4, 4> m_matrix_Q; // motion covariance matrix
        Eigen::Matrix<double, 2, 2> m_matrix_R; // measurement covariance matrix

        double m_accel_x;   // most recent acceleration measurement x
        double m_accel_z;   // most recent accelertaion measurement z

        double m_estimator_time{ros::Time::now().toSec()};

        const static double GRAVITY;  
        

};

#endif /* ESIMATOR */
