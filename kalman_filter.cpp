
#include <Eigen/Dense>

using Eigen::MatrixXf;
using Eigen::VectorXf;

class KalmanFilter{
  MatrixXf A;
  MatrixXf B;
  MatrixXf C;
  MatrixXf Q;
  MatrixXf R;
  MatrixXf I;
  VectorXf x;
  VectorXf xhat;
  MatrixXf P;
  MatrixXf Phat;

  public:
    KalmanFilter(int num_imus, double dt, double process_noise_constant, double measurement_noise_constant,double initial_noise_constant)
    {
      x = initX();
      P = initP(initial_noise_constant);
      A = initA(dt);
      B = initB();
      C = initC(num_imus);
      Q = initQ(process_noise_constant);
      R = initR(num_imus,measurement_noise_constant);
      I = MatrixXf::Identity(3,3);

    }
    void predict()
    {
    xhat = A * x; //currently no control added - else + B*u
    Phat = A * P * A.transpose() + Q;
    }
    void correct(VectorXf z)
    {
    MatrixXf K = (Phat * C.transpose()) * (C * Phat * C.transpose() + R).inverse();
    x = xhat + K * (z - C*xhat);
    P = (I - K*C)*Phat;
    }
    void filter(VectorXf z){
    predict();
    correct(z);
    }
    VectorXf getState(){
      return x;
    }
    MatrixXf getCovariance(){
      return P;
    }
// private methods
  MatrixXf initA(double dt){
    // creates a kinematic matrix with x updated as x + xdot * dt + 1/2 *xdotdot * dt^2
    A = MatrixXf::Identity(3,3);
    A(0,1) = dt;
    A(0,2) = 0.5 * dt * dt;
    A(1,2) = dt;
    return A;
  }
  MatrixXf initB(){
    // currently a placeholder - not implemented
    return MatrixXf::Identity(3,3);
  }
  VectorXf initX(){
    return VectorXf::Zero(3);
  }
  MatrixXf initP(double initial_noise_constant){
    // initialize P as diagonal matrix with initial_noise_constants on the diagonals
    return MatrixXf::Identity(3,3) * initial_noise_constant;
  }
  MatrixXf initC(int num_imus){
    // C is 3xnum_imus with 1s down the final column (as imus only measure acceleration)
    C = MatrixXf::Zero(3,num_imus);
    for (int i = 0; i<3; i++){
      C(i, num_imus-1) = 1.0;
    }
  return C; 
  }
  MatrixXf initQ(double process_noise_constant){
    // Q assumed (for now) to be diagonal with constant process noise down diagonal
   return  MatrixXf::Identity(3,3) * process_noise_constant;
  }
  MatrixXf initR(int num_imus, double measurement_noise_constant){
    // R is assumed to be diagonal (sensor noises are independent with constant measurement noise down diagonal
    return MatrixXf::Identity(num_imus, num_imus) * measurement_noise_constant;
  }
};


int main(){
  return 0;
  }

