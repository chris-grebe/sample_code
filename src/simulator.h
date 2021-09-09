/**
 * @brief Simulator simulates baloon system and publishes ground truth position and acceleration measurements
 * @author Christopher Grebe <christopher.grebe@robotics.utias.utoronto.ca>
 */
#ifndef SIMULATOR
#define SIMULATOR

#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include <math.h>
#include <Eigen/Dense>
#include <random>

class Simulator 
{
    public:
        /**
         * @brief Constructor
         * @param cd drag coeffient [unitless]
         * @param rho
         * @param mass
         * @param gravity gravity [m/s^2]
         * @param ac
         * @param gt_rate frequency to publish the ground truth measurements [Hz]
         * @param state_initial initial state [x position [m]; z position [m]; x velocity [m/s]; z velocity [m/s]]
         * @return
         */ 
        explicit Simulator(double cd, double rho, double mass, double gravity, double ac, double gt_rate, Eigen::Matrix<double, 4, 1> state_initial);
        virtual ~Simulator();

        /**
         * @brief propogates the state forward by the specified update rate using true dynamics
         * @param state the current state of the system. This will be updated by reference [x position [m]; z position [m]; x velocity [m/s]; z velocity [m/s]]
         * @param true_meas measurement to be updated by reference. [x position [m]; z position [m]; x acceleration [m/s^2]; z acceleration [m/s^2]]
         * @return no return value - state and true_meas are updated by reference
         */ 
        void updateState(Eigen::Matrix<double, 4, 1>& state, Eigen::Matrix<double, 4, 1>& true_meas);
       
        /**
         * @brief publishes ground truth measurements from gps and accelerometer
         * @param n
         * @return
         */ 
        void publishGroundTruth(ros::NodeHandle n);

    private:
        
        Eigen::Matrix<double, 4, 1> m_state_initial; // initial state [x position [m]; z position [m]; x velocity [m/s]; z velocity [m/s]]
        double m_cd;            // drag coefficient [unitless]
        double m_rho;           // []
        double m_m;             // mass []
        double m_g;             // gravity [m/s^2]
        double m_ac;            // []
        double m_gt_rate;       // frequency to publish the ground truth measurements [Hz]      

        std::default_random_engine m_generator;
        std::normal_distribution<double> m_distribution;  

        const static double STD_DEV; // process noise standard deviation
};
#endif /* SIMULATOR */
