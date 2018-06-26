#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

# define PI 3.14159265358979323846

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {

    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

    // Perform Measurment Update:
    VectorXd z_predicton = H_ * x_;
    VectorXd y = z - z_predicton; // y = z - H*x'
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    // Estimate New State:
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    // Predict the state vector by converting to polar:
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3); 

    //Compute polar coordinates of position:
    float rho = sqrt(px*px + py*py);
    float theta = atan2(py,px);
    float rho_dot;

    //Avoid divide by zero:
    if(fabs(rho) < 0.0001){
        rho_dot = 0;
    } else{
        rho_dot = (px*vx + py*vy) / rho;
    }
    
    // Store measurment vector h(x')
    VectorXd z_prediction = VectorXd(3);
    z_prediction << rho,theta,rho_dot;
    //Perform Measurement Update:    
    VectorXd y = z - z_prediction; // y = z - h(x')

    //Adjust delta phi to fit between -pi and pi
    float delta_phi = y(1);
    if(delta_phi >PI){y(1)-=2*PI;}
    if(delta_phi <(-PI)){y(1)+=2*PI;}


    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K = P_ * Ht * Si;

    //Calculate New State:
    x_ = x_ + (K*y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K*H_)*P_;
}
