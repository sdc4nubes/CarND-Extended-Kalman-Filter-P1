#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    previous_timestamp_ = 0;
    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_      = MatrixXd(3, 4);
    // Measurement covariance matrix - laser
    R_laser_ << 0.0225, 0     ,
                0     , 0.0225;
    // Measurement covariance matrix - radar
    R_radar_ << 0.09, 0     , 0   ,
                0   , 0.0009, 0   ,
                0   , 0     , 0.09;

    // measurement function matrix - laser
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
    // State covariance matrix P: low values for high certainty
    MatrixXd P_ = MatrixXd(4, 4);
    P_ << 1, 0, 0   , 0   ,
          0, 1, 0   , 0   ,
          0, 0, 1000, 0   ,
          0, 0, 0   , 1000;
    // State transition matrix F
    MatrixXd F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;
    // Covariance matrix Q based on noise vector
    MatrixXd Q_ = MatrixXd(4, 4);
    Q_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          1, 0, 1, 0,
          0, 1, 0, 1;
    // Input vector x
    VectorXd x_ = VectorXd(4);
    x_ << 1, 1, 1, 1;
    // Initialize the Fusion EKF (using laser matrices)
    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
}
/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

    /*****************************************************************************
    *  Initialize
    *****************************************************************************/
    if (!is_initialized_) {
        // Initialize ekf_.x
        // cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;
        // Initialize position and velocity
        float px = 0.0;
        float py = 0.0;
        float vx = 0.0;
        float vy = 0.0;
        // Radar
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            // cout << "EKF : First measurement RADAR" << endl;

            // Get polar coordinates
            float rho = measurement_pack.raw_measurements_[0]; // range
            float phi = measurement_pack.raw_measurements_[1]; // bearing
            float rho_dot = measurement_pack.raw_measurements_[2]; // velocity of rho
            // Convert coordinates from polar to Cartesian
            px = (rho * cos(phi) >= 0.0001 ? rho * cos(phi) : 0);
            py = (rho * sin(phi) >= 0.0001 ? rho * sin(phi) : 0);
            vx = rho_dot * cos(phi);
            vy = rho_dot * sin(phi);
        }
        // Laser
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            // cout << "EKF : First measurement LASER" << endl;
            // Get Cartesian coordinates
            px = measurement_pack.raw_measurements_[0];
            py = measurement_pack.raw_measurements_[1];
        }

        // Initialize the state ekf_.x_ with the first measurement
        ekf_.x_ << px, py, vx, vy;

        // Save first time stamp
        previous_timestamp_ = measurement_pack.timestamp_ ;
        is_initialized_ = true;
        return;
    }

    /*****************************************************************************
    *  Predict
    *****************************************************************************/
    // Get elapsed time since last measurement
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;
    // Update the state transition matrix F according to the new elapsed time
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;
    float dt_2   = dt * dt; // dt ^ 2
    float dt_3_2 = dt_2 * dt / 2.0; // dt ^ 3 / 2
    float dt_4_4 = dt_3_2 * dt / 2.0; // dt ^ 4 / 4
    // Set the acceleration noise components
    float noise_ax = 9;
    float noise_ay = 9;
    // Set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4_4 * noise_ax, 0                , dt_3_2 * noise_ax, 0                ,
               0                , dt_4_4 * noise_ay, 0                , dt_3_2 * noise_ay,
               dt_3_2 * noise_ax, 0                , dt_2 * noise_ax  , 0                ,
               0                , dt_3_2 * noise_ay, 0                , dt_2 * noise_ay  ;
    ekf_.Predict();

    /*****************************************************************************
    *  Update
    *****************************************************************************/
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        Hj_     = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.UpdateEKFR(measurement_pack.raw_measurements_);
    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.UpdateEKFL(measurement_pack.raw_measurements_);
    }
    // print the output
    // cout << "x_ = " << ekf_.x_ << endl;
    // cout << "P_ = " << ekf_.P_ << endl;
}
