#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen3/Dense"

class KalmanFilter{
public:
    //Constructor
    KalmanFilter(){
        is_initialized_=false;
    }

    //Destructor
    ~KalmanFilter(){}

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

    void MeasurementUpdate(const Eigen::VectorXd &z);

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
void KalmanFilter::Prediction(){
    x_=F_*x_;
    Eigen::MatrixXd Ft=F_.transpose();
    P_=F_*P_*Ft+Q_;
}
void KalmanFilter::MeasurementUpdate(const Eigen::VectorXd &z){
    Eigen::VectorXd y=z-H_*x_;
    MatrixXd S=H_*P_*H_.transpose()+R_;
    MatrixXd K=P_*H_.transpose()*S.inverse();
    x_=x_+(K*y);
    int size=x_.size();
    MatrixXd I=MatrixXd::Identity(size,size);
    P_=(I-K*H_)*P_;
}
