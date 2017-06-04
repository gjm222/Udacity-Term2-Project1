#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  float py = 0.0;
    float px = 0.0;
    float vy = 0.0;
    float vx = 0.0;
    
	VectorXd rmse(4);
	rmse << 0,0,0,0;

    // check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	if( estimations.size() == 0 ) {
	    cout << "Error: estimations are zero size" << endl;
	    return rmse;
	}    
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if( estimations.size() != ground_truth.size() ) {
	    cout << "Error: estimations and ground_truths matrices are not same dimensions" << endl;
	    return rmse;
	}    

	//accumulate squared residuals
	for(int i=0; i < estimations.size(); ++i){
        // ... your code here
        
        //cout << estimations.at(i) << endl;
        
        VectorXd xe = estimations.at(i);
        VectorXd xt = ground_truth.at(i);
        
        px = px + pow(xe(0) - xt(0),2);
        py = py + pow(xe(1) - xt(1),2);
        vx = vx + pow(xe(2) - xt(2),2);
        vy = vy + pow(xe(3) - xt(3),2);
	
	}

	//calculate the mean
	// ... your code here
	px = px / estimations.size();
	py = py / estimations.size();
	vx = vx / estimations.size();
	vy = vy / estimations.size();

	//calculate the squared root
	// ... your code here
	rmse(0) = sqrt(px);
	rmse(1) = sqrt(py);
	rmse(2) = sqrt(vx);
	rmse(3) = sqrt(vy);
	

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

    MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);
	float px2pluspy2 = 0.0;
	float vxpyminusvypx = 0.0;

	//Initialize
	Hj.setZero();
	
	//Pre-calculate
	px2pluspy2 = pow(px,2) + pow(py,2);
	vxpyminusvypx = vx * py - vy * px;
	
	//check division by zero
	if( px2pluspy2 == 0.0 ) {
	    cout << "error: divide by zero" << endl;
	}
	else {
	
	    Hj(0,0) = px / sqrt(px2pluspy2);
	    Hj(0,1) = py / sqrt(px2pluspy2);
	    
	    Hj(1,0) = (py/px2pluspy2) * -1;
	    Hj(1,1) = (px/px2pluspy2);
	    
	    Hj(2,0) = py * (vx * py - vy * px) / pow(px2pluspy2, 1.5);
	    Hj(2,1) = px * (vy * px - vx * py) / pow(px2pluspy2, 1.5);
	    Hj(2,2) = Hj(0,0);
	    Hj(2,3) = Hj(0,1);
	    
	}
	return Hj;
}
