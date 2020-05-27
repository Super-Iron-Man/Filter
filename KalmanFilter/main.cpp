#include "kalman_filter.h"
#include <iostream>
int main(){
    double m_x=0.0,m_y=0.0;
    double last_timestamp=0.0,now_timestamp=0.0;
    KalmanFilter kf;
    while(GetLidarData(m_x,m_y,now_timestamp)){
        //initialize kalman filter
        if(!kf.IsInitialized()){
            last_timestamp=now_timestamp;
            Eigen::VectorXd x_in(4,1);
            x_in << m_x,m_y,0.0,0.0;
            kf.Initialization(x_in);
            //state covariance matrix
            Eigen::MatrixXd P_in(4,4);
            P_in << 1.0,0.0,0.0,0.0,
                    0.0,1.0,0.0,0.0,
                    0.0,0.0,100.0,0.0,
                    0.0,0.0,0.0,100.0;
            kf.SetP(P_in);
            //process covariance matrix
            Eigen::MatrixXd Q_in(4,4);
            Q_in << 1.0,0.0,0.0,0.0,
                    0.0,1.0,0.0,0.0,
                    0.0,0.0,1.0,0.0,
                    0.0,0.0,0.0,1.0;
            kf.SetQ(Q_in);
            //measurement matrix
            Eigen::MatrixXd H_in(2,4);
            H_in << 1.0,0.0,0.0,0.0,
                    0.0,1.0,0.0,0.0;
            kf.SetH(H_in);
            //measurement covariance matrix
            //R is provided by Sensor supplier, in dataset
            Eigen::matrixXd R_in(2,2);
            R_in << 0.0225,0.0,
                    0.0,0.0225;
            kf.SetR(R_in);
            continue;
        }
        //state transition matrix
        double delta_t=now_timestamp-last_timestamp;
        last_timestamp=now_timestamp;
        Eigen::MatrixXd F_in(4,4);
        F_in << 1.0,0.0,delta_t,0.0,
                0.0,1.0,0.0,delta_t,
                0.0,0.0,1.0,0.0,
                0.0,0.0,0.0,1.0;
        kf.SetF(F_in);
        kf.Prediction();
        //measurement value
        Eigen::VectorXd z(2,1);
        z << m_x,m_y;
        kf.MeasurementUpdate(z);
        //get result
        Eigen::VectorXd x_out=kf.GetX();
        std::cout << "kalman output x: " << x_out(0) << "y: " << x_out(1) << std::endl;
    }
}