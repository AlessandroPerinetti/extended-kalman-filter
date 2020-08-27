#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;
using std::cout;
using std::endl;

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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 	0,
        	  0, 		0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 		0,
       		  0, 	0.0009, 0,
        	  0, 	0, 		0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  
  // Laser measurement matrix (since laser is only able to measure the position)
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // Initial covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 	0, 		0, 		0,
             0, 	1,		0, 		0,
             0, 	0,		1000, 	0,
             0, 	0, 		0,		1000;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {  													// if it is the first measurement, initialization is needed
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */
    
    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {		 // if the first measure comes from Radar sensor
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      
      float rho = measurement_pack.raw_measurements_[0]; 					// extract the radial distance
  	  float phi = measurement_pack.raw_measurements_[1]; 					// extract the bearing angle
  	  float rho_dot = measurement_pack.raw_measurements_[2];				// extract the radial velocity
            
      // Normalize phi in the range [-pi, pi]
      while (phi > M_PI)  phi -= 2.0 * M_PI;
      while (phi < -M_PI) phi += 2.0 * M_PI;
      
      // Convert polar coordinates into cartesian coordinates
      float px = rho * cos(phi);
      float py = rho * sin(phi);
  	  float vx = rho_dot * cos(phi);
  	  float vy = rho_dot * sin(phi);
      
      ekf_.x_ << px, py, vx , vy; 											// initialize the state vector
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
		float px = measurement_pack.raw_measurements_[0]; 					// extract x position
        float py = measurement_pack.raw_measurements_[1]; 					// extract x position
      	ekf_.x_  << px, py, 0, 0; 											// initialize the state vector
    }

    // If x and y position are < than a very small number
    if (fabs(ekf_.x_(0)) < 0.0001 and fabs(ekf_.x_(1)) < 0.0001){
      ekf_.x_(0) = 0.0001;
      ekf_.x_(1) = 0.0001;
    }
    
	// Keep the value on the previous timestamp in memory
    previous_timestamp_ = measurement_pack.timestamp_ ;
    
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
   // Define dt as the time difference in seconds between two consecutive observations
   float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  
   // Update the value on the previous timestamp in memory
   previous_timestamp_ = measurement_pack.timestamp_;

  // Compute the state transition matrix F using the evalued dt  
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 	0, 		dt, 	0,
             0, 	1, 		0, 		dt,
             0, 	0, 		1, 		0,
             0, 	0, 		0, 		1;

  // Compute the process noise covariance matrix Q
  float noise_ax = 9.0;														// Define longitudinal acceleration noise
  float noise_ay = 9.0;														// Define lateral acceleration noise

  float dt_2 = dt * dt; 													// Pre-calculate dt^2
  float dt_3 = dt_2 * dt; 													// Pre-calculate dt^3
  float dt_4 = dt_3 * dt; 													// Pre-calculate dt^4

  ekf_.Q_ = MatrixXd(4, 4);													// Evaluate matrix Q
  ekf_.Q_ << dt_4/4 * noise_ax, 	0, 						dt_3/2 * noise_ax, 		0,
	         0,						dt_4/4 * noise_ay,		0, 						dt_3/2 * noise_ay,
	         dt_3/2 * noise_ax, 	0, 						dt_2 * noise_ax,		0,
 	         0, 					dt_3/2 * noise_ay, 		0, 						dt_2 * noise_ay;


  ekf_.Predict();															// Call the prediction function

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {			// if the first measure comes from Radar sensor
	// TODO: Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);								// need to find the linearized H matrix evaluating its Jacobian through the specific function
  	ekf_.R_ = R_radar_;														// defined in the initialization 
  	ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {																	// if the first measure comes from Laser sensor
    // TODO: Laser updates
    ekf_.H_ = H_laser_;														// defined in the initialization 
  	ekf_.R_ = R_laser_;														// defined in the initialization 
  	ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
