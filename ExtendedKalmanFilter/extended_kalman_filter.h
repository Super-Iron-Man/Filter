#pragma once

#include "eigen3/Dense"

class ExtendedKalmanFilter{
public:
    //constructor
    ExtendedKalmanFilter(){
        is_initialized_=false;
    }
    //destructor
    ~ExtendedKalmanFilter(){}

    Eigen::VectorXd GetX(){
        return x_;
    }
    bool IsInitialized(){
        return is_initialized_;
    }
    void Initialization(Eigen::VectorXd x_in){
        x_=x_in;
        is_initialized_=true;
    }
    void SetF(Eigen::MatrixXd F_in){
        F_=F_in;
    }
    void SetP(Eigen::MatrixXd P_in){
        P_=P_in;
    }
    void SetQ(Eigen::MatrixXd Q_in){
        Q_=Q_in;
    }
    void SetH(Eigen::MatrixXd H_in){
        H_=H_in;
    }
    void SetR(Eigen::MatrixXd R_in){
        R_=R_in;
    }
    void Prediction();
    void CalculateJacobianMatrix();
    void MeasurementUpdate(const Eigen:VectorXd &z);
private:
    //flag of initialization
    bool is_initialized_;
    //state vector
    Eigen::VectorXd x_;
        //state transistion matrix
    Eigen::MatrixXd F_;
    //state covariance matrix
    Eigen::MatrixXd P_;
    //process covariance matrix
    Eigen::MatrixXd Q_;
    //measurement matrix
    Eigen::MatrixXd H_;
    //measuremnet covariance matrix
    Eigen::MatrixXd R_;
}
void ExtendedKalmanFilter::Prediction(){
    x_=F_*x_;
    Eigen::MatrixXd Ft=F_.transpose();
    P_=F_*P_*Ft+Q_;
}
void ExtendedKalmanFilter::CalculateJacobianMatrix(){
    MatrixXd Hj(3,4);
    //get state parameters
    float px=x_(0);
    float py=x_(1);
    float vx=x_(2);
    float vy=x_(3);
    //pre-compute a set of terms to avoid repeated calculation
    float c1=px*px+py*py;
    float c2=sqrt(c1);
    float c3=(c1*c2);
    //check division by zero
    if(fabs(c1)<0.0001){
        H_=Hj;
        return;
    }
    //compute the Jacobian matrix
    Hj << (px/c2),(py/c2),0,0,
          -(py/c1),(px/c1),0,0,
          py*(vx*py-vy*px)/c3,px*(px*vy-py*vx)/c3,px/c2,py/c2;
    H_=Hj;
    return;
}
void ExtendedKalmanFilter::MeasurementUpdate(const Eigen::VectorXd &z){
    double rho=sqrt(x_(0)*x_(0)+x_(1)*x_(1));
    double theta=atan2(x_(1),x_(0));
    double rho_dot=(x_(0)*x_(2)+x_(1)*x_(3))/rho;
    VectorXd h=VectorXd(3);
    h << rho,theta,rho_dot;

    VectorXd y=z-h;
    
    CalculateJacobianMatrix();

    MatrixXd S=H_*P_*H_.transpose()+R_;
    MatrixXd K=P_H_.transpose()*S.inverse();
    x_=x_+(K*y);
    MatrixXd I=MatrixXd::Identity(x_.size(),x_.size());
    P_=(I-K*H_)*P_;
}