#include <scan_matcher/transform.h>
#include <cmath>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <complex>

#define NUM_ITERATIONS 100

using namespace std;

void transformPoints(const vector<Point> &points, Transform &t, vector<Point> &transformed_points)
{
  transformed_points.clear();
  for (int i = 0; i < points.size(); i++)
  {
    transformed_points.push_back(t.apply(points[i]));
    //printf("%f %transformed_points.back().r, transformed_points.back().theta);
  }
}

// returns the largest real root to ax^3 + bx^2 + cx + d = 0
complex<float> get_cubic_root(float a, float b, float c, float d)
{
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<<";"<<std::endl;
  // Reduce to depressed cubic
  float p = c / a - b * b / (3 * a * a);
  float q = 2 * b * b * b / (27 * a * a * a) + d / a - b * c / (3 * a * a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;

  complex<float> xi(-.5, sqrt(3) / 2);

  complex<float> inside = sqrt(q * q / 4 + p * p * p / 27);

  complex<float> root;

  for (float k = 0; k < 3; ++k)
  {
    // get root for 3 possible values of k
    root = -b / (3 * a) + pow(xi, k) * pow(-q / 2.f + inside, 1.f / 3.f) + pow(xi, 2.f * k) * pow(-q / 2.f - inside, 1.f / 3.f);
    //std::cout<<"RootTemp: "<< root<<std::endl;
    if (root.imag() != 0)
    {
      return root;
    }
  }

  return root;
}

// returns the largest real root to ax^4 + bx^3 + cx^2 + dx + e = 0
float greatest_real_root(float a, float b, float c, float d, float e)
{
  // Written with inspiration from: https://en.wikipedia.org/wiki/Quartic_function#General_formula_for_roots
  //std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<< ";  e= " << e<<";"<<std::endl;

  // Reduce to depressed Quadratic
  float p = (8 * a * c - 3 * b * b) / (8 * a * a);
  float q = (b * b * b - 4 * a * b * c + 8 * a * a * d) / (8 * a * a * a);
  float r = (-3 * b * b * b * b + 256 * a * a * a * e - 64 * a * a * b * d + 16 * a * b * b * c) / (256 * a * a * a * a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;
  // std::cout<<"r = "<<r<<";"<<std::endl;

  // Ferrari's Solution for Quartics: 8m^3 + 8pm^2 + (2p^2-8r)m - q^2 = 0
  complex<float> m = get_cubic_root(8, 8 * p, 2 * p * p - 8 * r, -q * q);

  complex<float> root1 = -b / (4 * a) + (sqrt(2.f * m) + sqrt(-(2 * p + 2.f * m + sqrt(2.f) * q / sqrt(m)))) / 2.f;
  complex<float> root2 = -b / (4 * a) + (sqrt(2.f * m) - sqrt(-(2 * p + 2.f * m + sqrt(2.f) * q / sqrt(m)))) / 2.f;
  complex<float> root3 = -b / (4 * a) + (-sqrt(2.f * m) + sqrt(-(2 * p + 2.f * m - sqrt(2.f) * q / sqrt(m)))) / 2.f;
  complex<float> root4 = -b / (4 * a) + (-sqrt(2.f * m) - sqrt(-(2 * p + 2.f * m - sqrt(2.f) * q / sqrt(m)))) / 2.f;

  vector<complex<float>> roots{root1, root2, root3, root4};

  float max_real_root = 0.f;

  for (complex<float> root : roots)
  {
    if (root.imag() == 0)
    {
      max_real_root = max(max_real_root, root.real());
    }
    //std::cout<<"Max real root:" << max_real_root<<std::endl;

    return max_real_root;
  }
}

Eigen::Matrix4f merge_2x2_to_4x4_matrice(Eigen::Matrix2f m_0_0, Eigen::Matrix2f m_0_1, Eigen::Matrix2f m_1_0, Eigen::Matrix2f m_1_1)
{
  Eigen::Matrix4f res;

  for (int y = 0; y < 4; y++)
  {
    for (int z = 0; z < 4; z++)
    {
      if (y < 2 && z < 2)
        res(y, z) = m_0_0(y, z);
      else if (y < 2 && z < 4)
        res(y, z) = m_0_1(y, z - 2);
      else if (y < 4 && z < 2)
        res(y, z) = m_1_0(y - 2, z);
      else
        res(y, z) = m_1_1(y - 2, z - 2);
    }
  }

  return res;
}

void updateTransform(vector<Correspondence> &corresponds, Transform &curr_trans)
{
  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // You can use the helper functions which are defined above for finding roots and transforming points as and when needed.
  // use helper functions and structs in transform.h and correspond.h
  // input : corresponds : a struct vector of Correspondene struct object defined in correspond.
  // input : curr_trans : A Transform object refernece
  // output : update the curr_trans object. Being a call by reference function, Any changes you make to curr_trans will be reflected in the calling function in the scan_match.cpp program/

  // You can change the number of iterations here. More the number of iterations, slower will be the convergence but more accurate will be the results. You need to find the right balance.
  int number_iter = NUM_ITERATIONS;

  // Fill in the values for the matrices
  Eigen::Matrix4f M, W;
  Eigen::MatrixXf g(4, 1);

  M << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;

  W << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;

  g << 0, 0, 0, 0;

  for (int i = 0; i < number_iter && i < corresponds.size(); i++)
  {
    //fill in the values of the matrics
    Eigen::MatrixXf M_i(2, 4);
    Eigen::Matrix2f C_i;
    Eigen::Vector2f pi_i = corresponds[i].getPiVec();

    Eigen::Vector2f ni = corresponds[i].getNormalNorm();

    M_i << 1, 0, pi_i(0), -pi_i(1),
        0, 1, pi_i(1), pi_i(0);

    C_i << ni * ni.transpose(); // maybe consider the w_i

    M += M_i.transpose() * C_i * M_i;
    g += -2 * pi_i.transpose() * C_i * M_i;
  }

  // Define sub-matrices A, B, D from M
  Eigen::Matrix2f A, B, D;
  Eigen::Matrix4f M_temp = 2 * M;

  A << M_temp(0, 0), M_temp(0, 1),
      M_temp(1, 0), M_temp(1, 1);

  B << M_temp(0, 2), M_temp(0, 3),
      M_temp(1, 2), M_temp(1, 3);

  D << M_temp(2, 2), M_temp(2, 3),
      M_temp(3, 2), M_temp(3, 3);

  //define S and S_A matrices from the matrices A B and D
  Eigen::Matrix2f S = (D - B.transpose() * A.inverse() * B);
  ;
  Eigen::Matrix2f S_A = S.determinant() * S.inverse();
  ;

  //find the coefficients of the quadratic function of lambda
  float pow_2;
  float pow_1;
  float pow_0;
  Eigen::Matrix4f m_pow_2, m_pow_1, m_pow_0;

  Eigen::Matrix2f m0_0_0, m0_0_1, m0_1_0, m0_1_1;
  Eigen::Matrix2f m1_0_0, m1_0_1, m1_1_0, m1_1_1;
  Eigen::Matrix2f m2_0_0, m2_0_1, m2_1_0, m2_1_1;

  m2_0_0 << A.inverse() * B * B.transpose() * A.inverse().transpose();
  m2_0_1 << -A.inverse() * B;
  m2_1_0 << -A.inverse() * B;
  m2_1_1 << 1.0, 0.0,
      0.0, 1.0;

  m1_0_0 << A.inverse() * B * S_A * B.transpose() * A.inverse().transpose();
  m1_0_1 << -A.inverse() * B * S_A;
  m1_1_0 << -A.inverse() * B * S_A;
  m1_1_1 << S_A;

  m0_0_0 << A.inverse() * B * S_A.transpose() * S_A * B.transpose() * A.inverse().transpose();
  m0_0_1 << -A.inverse() * B * S_A.transpose() * S_A;
  m0_1_0 << -A.inverse() * B * S_A.transpose() * S_A;
  m0_1_1 << S_A.transpose() * S_A;

  m_pow_2 = merge_2x2_to_4x4_matrice(m2_0_0, m2_0_1, m2_1_0, m2_1_1);
  m_pow_1 = merge_2x2_to_4x4_matrice(m1_0_0, m1_0_1, m1_1_0, m1_1_1);
  m_pow_0 = merge_2x2_to_4x4_matrice(m0_0_0, m0_0_1, m0_1_0, m0_1_1);

  pow_2 = (4.0 * g * m_pow_2 * g.transpose())(0, 0); // TODO swapped g and g^T to get a valid matrix multiplication
  pow_1 = (4.0 * g * m_pow_1 * g.transpose())(0, 0);
  pow_0 = (g * m_pow_0 * g.transpose())(0, 0);

  // find the value of lambda by solving the equation formed. You can use the greatest real root function
  float lambda = greatest_real_root(0.0, 0.0, pow_2, pow_1, pow_0);

  //find the value of x which is the vector for translation and rotation
  Eigen::Vector4f x = -(2 * M + 2 * lambda * W).inverse().transpose() * g.transpose(); // TODO using g.transpose() instead of g to make matrix multiplication working

  // Convert from x to new transform
  float theta = atan2(x(3), x(2));
  curr_trans = Transform(x(0), x(1), theta);
}
