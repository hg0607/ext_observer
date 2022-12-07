#ifndef DOUBLE_LINK_H
#define DOUBLE_LINK_H

#include "../lib/external_observer.h" 
#include <cmath>

static double kDynamicsParams[13] = {28.567919,  1.510246,  10.735342, 60.761491,
                                     -96.242815, 25.440762,  0.906162, -15.770885, 41.802035,
                                     55.665622,  124.677992, 2.322097, 46.148291};

class DoubleLink : public RobotDynamics {
public:
  DoubleLink();
  
  // get inertia
  Matrix getM(Vector& q);
  // get Coriolis/centrifugal matrix
  Matrix getC(Vector& q, Vector& qd);
  // get GRAVITYity
  Vector getG(Vector& q);
  // friction model 
  Vector getFriction(Vector& qd);
  // number of joints
  int jointNo() { return 2; }
      
private:
  Matrix M, C, J;
  Vector G, fric;
  double m1,m2;
  double l1, l2;
  double lc1, lc2; 
  double I1, I2;
};

DoubleLink::DoubleLink() 
           : RobotDynamics()
           , M(Matrix(2,2))
           , C(Matrix(2,2))
           , J(Matrix(6,2))
           , G(Vector(2))    
           , fric(Vector(2)) 
{
  m1 = 1; m2 = 1;
  l1 = 0.5; l2 = 0.5;
  lc1 = 0.25; lc2 = 0.25;
  I1 = 0.3; I2 = 0.2;
  M.setZero(); 
  C.setZero(); 
  J.setZero(); 
  G.setZero(); 
  fric.setZero();
}

Matrix DoubleLink::getM(Vector& q)
{
  M(0,0) = kDynamicsParams[3];
  M(0,1) = 0;
  M(1,0) = 0;
  M(1,1) = kDynamicsParams[8]; 
  return M;
}

Matrix DoubleLink::getC(Vector& q, Vector& qd)
{
  double h = -m2*l1*lc2*sin(q(1)); 
  C(0,0) = 0;
  C(0,1) = 0;
  C(1,0) = 0; 
  return C;
}

Vector DoubleLink::getG(Vector& q)
{
  G(0) = 0;
  G(1) = kDynamicsParams[4] * cos(q[1]); 
  return G;
}

Vector DoubleLink::getFriction(Vector& qd)
{  
    fric[0] = tanh(5*qd[0]) * kDynamicsParams[0] + kDynamicsParams[1] * qd[0] + kDynamicsParams[2];
    fric[1] = tanh(5*qd[1]) * kDynamicsParams[5] + kDynamicsParams[6] * qd[1] + kDynamicsParams[7];
  return fric;
}

/*
Matrix DoubleLink::getJacobian(Vector& q)
{
  double s12 = sin(q(0)+q(1)), c12 = cos(q(0)+q(1));
  J(0,0) = -l1*sin(q(0))-l2*s12;
  J(1,0) = l1*cos(q(0)) + l2*c12;
  J(5,0) = 1;
  J(0,1) = -l2*s12;
  J(1,1) = l2*c12;
  J(5,1) = 1; 
  return J;
}
*/

#endif // DOUBLE_LINK_H
