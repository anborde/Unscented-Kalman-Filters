#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void NormalizingAngle(double& phi);

/**
* Initializes Unscented Kalman filter
* This is scaffolding, do not modify
*/
UKF::UKF() {
    
    // Opening NIS data files to save NIS data for LASER and RADAR
    nis_laser_file.open("NIS_laser.csv", ios::out);
    nis_radar_file.open("NIS_radar.csv", ios::out);
    
    // Providing Header for the data columns
    nis_laser_file << "NIS Value" <<endl;
    nis_radar_file << "NIS Value" <<endl;
    
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.3926;

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // Radar measurement noise standard deviation radius in m
    std_radr_ = 0.3;

    // Radar measurement noise standard deviation angle in rad
    std_radphi_ = 0.03;

    // Radar measurement noise standard deviation radius change in m/s
    std_radrd_ = 0.3;
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

    /**
    TODO:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */
    // Initializing is_initialized as false for the first measurement
    is_initialized_ = false;

    // State Dimension
    n_x_ = x_.size();

    // Augmented State Dimension
    n_aug_ = n_x_ + 2;

    // Sigma Point Spreading Parameter
    lambda_ = 3 - n_aug_;
    
    // Sigma Points
    Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ + 1);
    
    
    // Weights for Sigma Points
    weights_ = VectorXd(2*n_aug_+1);
    
    //set weights
    for (int i = 1; i <= 2*n_aug_+1; i++)
    {
        if(i == 1)
        {
            weights_(i-1) = lambda_/(lambda_ + n_aug_);
        }
        else
        {
            weights_(i-1) = 1/(2*(lambda_ + n_aug_));
        }
    }
    
    // Measuremet Matrix - Laser
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;
}

UKF::~UKF() {
    
    // Closing NIS data files
    nis_laser_file.close();
    nis_radar_file.close();
}

/**
* @param {MeasurementPackage} meas_package The latest measurement data of
* either radar or laser.
*/
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
    if (!is_initialized_) {
        // Initializing the state with the first measurement
        cout<< "UKF-->" << endl;
        
        if ( meas_package.sensor_type_ == MeasurementPackage::RADAR && use_laser_==true)
        {
            // If RADAR Measurement and we intend to use RADAR for our experiment
            
            // Converting Polar to Cartesian Co-ordinates and Initialising
            float range = meas_package.raw_measurements_[0];
            float bearing = meas_package.raw_measurements_[1];
            
            float px = range*cos(bearing);
            float py = range*sin(bearing);
           
            x_ << px, py, 0.0, 0.0, 0.0;
            
        }
        else if(meas_package.sensor_type_ == MeasurementPackage::LASER && use_radar_==true)
        {
            // If LASER Measurement and we intend to use LASER for our experiment
            float px = meas_package.raw_measurements_[0];
            float py = meas_package.raw_measurements_[1];
        
            x_ << px, py, 0.0, 0.0, 0.0;
        }
        else
        {
            return;
        }
        
        // Initializing Co-variance matrix
        P_ = MatrixXd::Identity(5,5);
        
        // Updating timestamp of last recorded data
        time_us_ = meas_package.timestamp_;
        
        // done initializing no need to predict or update
        is_initialized_ = true;
        
        
        return;
    }
    
    /*
        Checking whether received measurement is of a sensor type that is intended to be used for
        Prediction cycle of UKF
    */
    if ((meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true) || (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true))
    {
        // Calculating difference between timestamps dt in seconds
        double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
        time_us_ = meas_package.timestamp_;
        
        // Checking whether dt is not zero
        if (dt > 0.0)
        {
            // Predict
            Prediction(dt);
        }
        
    }
   
    
    /*
        Checking whether received measurement is of a sensor type that is intended to be used for
        Updation cycle of UKF
    */
    
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_==true)
    {
        // If RADAR measurement recieved & we intend to use it, calling UpdateRadar()
        UpdateRadar(meas_package);
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true)
    {
        // If LASER measurement recieved & we intend to use it, calling UpdateLaser()
        UpdateLidar(meas_package);
    }
}

/**
* Predicts sigma points, the state, and the state covariance matrix.
* @param {double} delta_t the change in time (in seconds) between the last
* measurement and this one.
*/
void UKF::Prediction(double delta_t) {
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
    
    // Generating Sigma Points
    
    // Augmenting the state and the co-variance matrix
    VectorXd x_aug = VectorXd(n_aug_);
    
    MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
    
    MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);
    
    x_aug.head(5) = x_;
    x_aug(5) = 0;
    x_aug(6) = 0;
    
    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(5,5) = std_a_*std_a_;
    P_aug(6,6) = std_yawdd_*std_yawdd_;
    
    // Calculating square root of P
    MatrixXd A = P_aug.llt().matrixL();
    
    Xsig_aug.col(0) = x_aug;
    
    // Generating Sigma Points for Augmented State and Co-variance Matrix
    for(int i = 0; i < n_aug_; i++)
    {
        Xsig_aug.col(i + 1) = x_aug + A.col(i)*sqrt(lambda_ + n_aug_);
        Xsig_aug.col(i+1+n_aug_) = x_aug - A.col(i)*sqrt(lambda_ + n_aug_);
    }
    
    // Predicting Sigma Points
   
    // Temporary vector used to calculate the predicted sigma points
    VectorXd temp;
   
    for(int i = 0; i<2*n_aug_ + 1; i++)
    {
        temp = Xsig_aug.col(i).head(n_x_);
        
        const double v = temp(2);
        const double yaw = temp(3);
        const double yawrate = temp(4);
        const double nu_a = Xsig_aug(5,i);
        const double nu_yaw = Xsig_aug(6,i);
        
        
        if (yawrate == 0)
        {
            temp(0) += v*cos(yaw)*delta_t;
            temp(1) += v*sin(yaw)*delta_t;
        }
        else
        {
            temp(0) += (v/yawrate)*(sin(yaw + delta_t*yawrate) - sin(yaw));
            temp(1) += (v/yawrate)*(-cos(yaw + delta_t*yawrate) + cos(yaw));
        }
        
        temp(3) += yawrate*delta_t;
        
        // adding noise
        
        temp(0) += 0.5*pow(delta_t, 2)*cos(yaw)*nu_a;
        temp(1) += 0.5*pow(delta_t, 2)*sin(yaw)*nu_a;
        temp(2) += delta_t*nu_a;
        temp(3) += 0.5*pow(delta_t,2)*nu_yaw;
        temp(4) += delta_t*nu_yaw;
        
        Xsig_pred_.col(i) = temp;
    }
    
    // Predicting Mean & Co-variance
    
    //predict state mean
    x_ = Xsig_pred_*weights_;
    
    //predict state covariance matrix
    P_.fill(0.0);
    
    for (int i = 0; i < 2*n_aug_+1; i++)
    {
        VectorXd temp = Xsig_pred_.col(i) - x_;
        
        while (temp(3)> M_PI)
        {
            temp(3)-=2.*M_PI;
        }
        while (temp(3)<-M_PI)
        {
            temp(3)+=2.*M_PI;
        }
        
        P_ += weights_(i)*temp*temp.transpose();
    }
}

/**
* Updates the state and the state covariance matrix using a laser measurement.
* @param {MeasurementPackage} meas_package
*/
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */
    
    //set measurement dimension, laser can measure px and py
    int n_z = 2;
    
    //mean predicted measurement
    VectorXd z_pred = H_laser_*x_;
    
    // Vector to store the actual state
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_;
    
    VectorXd y = z - z_pred;
    
    //measurement covariance matrix S
    MatrixXd S = H_laser_ * P_ * H_laser_.transpose();
    
 
    // Adding Measurement Co-variance Noise
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<    std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;
    
    S = S + R;
    
    //calculate Kalman gain K;
    MatrixXd K = P_ * H_laser_.transpose() * S.inverse();
    
  
    //update state mean and covariance matrix
    x_ = x_ + K*y;
   
    MatrixXd I = MatrixXd::Identity(n_x_, n_x_);
    
    P_ = (I - K*H_laser_)*P_;
    
     // Calculating NIS value 'epsilon' and writing to the file
    double epsilon = y.transpose()*S.inverse()*y;
    nis_laser_file<<epsilon<<endl;
}

/**
* Updates the state and the state covariance matrix using a radar measurement.
* @param {MeasurementPackage} meas_package
*/
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
    
    //set measurement dimension, radar can measure r, phi, and r_dot
    const int n_z = 3;

    //create matrix for sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
    
    //mean predicted measurement
    VectorXd z_pred = VectorXd(n_z);
    
    //measurement covariance matrix S
    MatrixXd S = MatrixXd(n_z,n_z);
    
    //transform sigma points into measurement space
    for(int i = 0; i<2*n_aug_ + 1; i++)
    {
        double px = Xsig_pred_(0,i);
        double py = Xsig_pred_(1, i);
        double v = Xsig_pred_(2, i);
        double psi = Xsig_pred_(3, i);
        
        double r = sqrt((pow(px,2) + pow(py, 2)));
        double phi = atan2(py, px);
        double rd = ((px*cos(psi)*v) + (py*sin(psi)*v))/sqrt((pow(px,2) + pow(py, 2)));
        
        Zsig.col(i) << r, phi, rd;
    }
    
    //calculate mean predicted measurement
    z_pred = Zsig*weights_;
    
    NormalizingAngle(z_pred(1));
    
    //calculate innovation covariance matrix S
    
    S.fill(0.0);
    
    for (int i = 0; i < 2*n_aug_+1; i++)
    {
        VectorXd temp = Zsig.col(i) - z_pred;
        
        NormalizingAngle(temp(1));
        
        S = S + weights_(i)*temp*temp.transpose();
    }
    
    // Adding Measurement Co-variance Noise
    MatrixXd R = MatrixXd(n_z, n_z);
    R <<    std_radr_*std_radr_, 0, 0,
            0, std_radphi_*std_radphi_, 0,
            0, 0, std_radrd_*std_radrd_;
    
    S = S + R;
    
    // Vector to store the actual state
    VectorXd z = VectorXd(n_z);
    z << meas_package.raw_measurements_;
    
    //create matrix for cross correlation Tc
    MatrixXd Tc = MatrixXd(n_x_, n_z);

    Tc.fill(0.0);
    for(int i = 0; i < 2*n_aug_ + 1; i++)
    {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        VectorXd z_diff = Zsig.col(i) - z_pred;
        
        NormalizingAngle(x_diff(1));
        
        NormalizingAngle(z_diff(1));
        
        Tc += weights_(i) * x_diff * z_diff.transpose();
    }
    
    //calculate Kalman gain K;
    MatrixXd K = Tc*S.inverse();
    
    //update state mean and covariance matrix
    VectorXd y = z - z_pred;
    
    NormalizingAngle(y(1));
    
    x_ = x_ + K*y;
    P_ = P_ - K*S*K.transpose();
    
    // Calculating NIS value 'epsilon' and writing to the file
    double epsilon = y.transpose()*S.inverse()*y;
    nis_radar_file<<epsilon<<endl;
}

void NormalizingAngle(double& phi)
{
    phi = atan2(sin(phi), cos(phi));
}
