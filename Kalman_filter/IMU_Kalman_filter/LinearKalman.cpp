#include "KalmanFilterAbstract.hpp"

class LinearKalmanFilter : public KalmanFilter {
public:
 // Constructor
 LinearKalmanFilter(
    double dt,
    const MatrixXd& F,
    const MatrixXd& H,
    const MatrixXd& Q, 
    const MatrixXd& R,
    const MatrixXd& P, 
    MatrixXd* B = nullptr
    ) : KalmanFilter(dt, F, H, Q, R, P, B) {}

 // Override predict method
 void predict() override {
  // Implementation of predict method for LinearKalmanFilter
 }

 // Override update method
 void update() override {
  // Implementation of update method for LinearKalmanFilter
 }
};
