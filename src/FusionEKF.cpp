#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <cmath>

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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
		  	  0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
		  	  0, 0.0009, 0,
			  0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1,0,0,0,
		      0,1,0,0;

  ekf_.H_ = H_laser_;

  ekf_.R_laser_ = R_laser_;
  ekf_.R_radar_ = R_radar_;


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


	cout << "Start x_ = " << ekf_.x_ << endl;
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    ekf_.P_ = MatrixXd(4,4);

    ekf_.P_ << 1, 0, 0, 0,
         	   0, 1, 0, 0,
           	   0, 0, 1000,0,
           	   0, 0, 0, 1000;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	ekf_.x_ << measurement_pack.raw_measurements_[0]*cos(float(measurement_pack.raw_measurements_[1])),
    			measurement_pack.raw_measurements_[0]*sin(float(measurement_pack.raw_measurements_[1])),
				measurement_pack.raw_measurements_[2]*cos(float(measurement_pack.raw_measurements_[1])),
				measurement_pack.raw_measurements_[2]*sin(float(measurement_pack.raw_measurements_[1]));

    	previous_timestamp_ = measurement_pack.timestamp_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0,0;

    	previous_timestamp_ = measurement_pack.timestamp_;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  double dt = double(measurement_pack.timestamp_ - previous_timestamp_)/1000000;
  previous_timestamp_ = measurement_pack.timestamp_;
  double dt_2 = dt*dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;


  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, dt, 0,
		  	 0, 1,  0 , dt,
  	  	  	 0, 0,  1 , 0,
			 0, 0,  0 , 1;

  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << 0.25*dt_4*9, 0, 0.5*dt_3*9, 0,
  		  	 0, 0.25*dt_4*9, 0, 0.5*dt_3*9,
			 0.5*dt_3*9, 0,  dt_2*9 , 0,
  			 0, 0.5*dt_3*9,  0 , dt_2*9;

  ekf_.Predict();

  cout << "After Pred. x_ = " << ekf_.x_ << endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	ekf_.Hj_ = tools.CalculateJacobian(ekf_.x_);
	ekf_.UpdateEKF(measurement_pack.raw_measurements_);


  } else {
    // Laser updates
	ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
