#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools()
{}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd rmse(4);
    rmse << 0,0,0,0;
    
    if (estimations.size() == 0 || estimations.size() != ground_truth.size())
    {
        return rmse;
    }
    
    //accumulating squared residuals
    for(int i=0; i < estimations.size(); ++i)
    {
        VectorXd temp = estimations[i] - ground_truth[i];
        temp = temp.array()*temp.array();
        rmse += temp;
    }
    
    //calculating the mean
    rmse = rmse/estimations.size();
    
    //calculating rmse
    rmse = rmse.array().sqrt();
    
    cout<<"RMS"<<endl;
    cout<<"------------------------------------"<<endl;
    cout<<"px:"<<rmse[0]<<endl;
    cout<<"py:"<<rmse[1]<<endl;
    cout<<"vx:"<<rmse[2]<<endl;
    cout<<"vy:"<<rmse[3]<<"\n"<<endl;
    
    return rmse;
}
