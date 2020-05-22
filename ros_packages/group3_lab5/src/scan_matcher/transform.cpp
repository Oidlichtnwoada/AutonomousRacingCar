#include <scan_matcher/transform.h>
#include <complex>
#include <unsupported/Eigen/Polynomials>

Transform::Transform() : x_disp(0), y_disp(0), theta_rot(0) {}

Transform::Transform(float _x_disp, float _y_disp, float _theta_rot) :
        x_disp(_x_disp), y_disp(_y_disp), theta_rot(_theta_rot) {}

bool Transform::operator==(const Transform& t2) const {
    return std::abs(x_disp-t2.x_disp) < EPSILON &&
           std::abs(y_disp-t2.y_disp) < EPSILON &&
           std::abs(theta_rot - t2.theta_rot) < EPSILON;
}

bool Transform::operator!=(const Transform& t2) const {
    return !(*this == t2);
}

Transform Transform::operator+(const Transform& t2) const {
    const Eigen::Matrix3f H1 = getMatrix();
    const Eigen::Matrix3f H2 = t2.getMatrix();
    const Eigen::Matrix3f H = H1*H2;
    const float x = H(0,2);
    const float y = H(1,2);
    const float theta = std::atan2(H(1,0),H(0,0));
    return Transform(x, y, theta);
}

Point Transform::apply(const Point _p) const {
    const float x = _p.getX() + x_disp;
    const float y = _p.getY() + y_disp;
    Point p;
    p.r = std::sqrt(x*x+y*y);
    p.theta = std::atan2(y,x);
    p.theta = p.theta + theta_rot;
    return p;
}

Eigen::Matrix3f Transform::getMatrix() const {
    Eigen::Matrix3f mat;
    mat << cos(theta_rot), -sin(theta_rot), x_disp, sin(theta_rot), cos(theta_rot), y_disp, 0, 0, 1;
    return mat;
}

Transform Transform::inverse() const {
    return(Transform(-x_disp, -y_disp, -theta_rot));
}

std::vector<Point> transformPoints(const std::vector<Point> &points, const Transform &t)
{
    std::vector<Point> transformed_points;
    for (int i = 0; i < points.size(); i++) {
        transformed_points.push_back(t.apply(points[i]));
    }
    return(transformed_points);
}

double find_greatest_real_root(Eigen::VectorXd poly_coeffs) {
    const int n = poly_coeffs.size();
    assert(n==5);
    Eigen::PolynomialSolver<double, 4> psolve(poly_coeffs);
    Eigen::Matrix<std::complex<double>, 4, 1, 0, 4, 1> eigen_roots = psolve.roots();
    int assigned = 0;
    double lambda = 0;
    for(unsigned int i=0; i<eigen_roots.size(); i++){
        if(eigen_roots(i).imag() == 0){
            if(!assigned || eigen_roots(i).real() > lambda) {
                assigned = 1;
                lambda = eigen_roots(i).real();
            }
        }
    }
    assert(assigned);
    return(lambda);
}

Eigen::VectorXd conv(const Eigen::VectorXd& f, const Eigen::VectorXd& g) {
    // conv(f,g) returns the full  convolution of vectors f and g
    // Adapted from: http://coliru.stacked-crooked.com/a/d4371c490934d34e
    int const nf = f.size();
    int const ng = g.size();
    int const n  = nf + ng - 1;
    Eigen::VectorXd out(n);
    out.fill(0.f);
    for(auto i(0); i < n; ++i) {
        int const jmn = (i >= ng - 1)? i - (ng - 1) : 0;
        int const jmx = (i <  nf - 1)? i            : nf - 1;
        for(auto j(jmn); j <= jmx; ++j) {
            out[i] += (f[j] * g[i - j]);
        }
    }
    return out;
}

Transform estimateTransformation(const SimpleCorrespondences& correspondences) {

    // Estimate transformation from t0 to t1

    Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
    Eigen::Vector4d g = Eigen::Vector4d::Zero();
    for(unsigned int i=0; i<correspondences.size(); i++) {
        
        const Eigen::Vector2d& p = correspondences[i].p_t0().cast<double>();
        const Eigen::Vector2d& q = correspondences[i].p_t1().cast<double>();
        const Eigen::Vector2d& nn = correspondences[i].nn().cast<double>();

        Eigen::Matrix2d C_i;
        C_i = nn * nn.transpose();

        Eigen::Matrix<double, 2, 4> M_i;
        M_i <<  1.0, 0.0, p.x(), -p.y(),
                0.0, 1.0, p.y(),  p.x();
        M = M + M_i.transpose() * C_i * M_i;
        g = g + (-2.0 * q.transpose() * C_i * M_i).transpose();
    }

    Eigen::Matrix4d W;
    W <<    0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    // M = [A B
    //      C D]

    M = 2.0 * M;
    const Eigen::Matrix2d A = M.block(0,0,2,2);
    const Eigen::Matrix2d B = M.block(0,2,2,2);
    //const Eigen::Matrix2d C = M.block(2,0,2,2); // not needed
    const Eigen::Matrix2d D = M.block(2,2,2,2);

    const Eigen::Matrix2d S = D - B.transpose() * A.inverse() * B;
    const Eigen::Matrix2d Sa = S.inverse() * S.determinant();

    const Eigen::Vector2d g1 = g.segment(0,2);
    const Eigen::Vector2d g2 = g.segment(2,2);

    Eigen::VectorXd poly_coefficients(5);
    poly_coefficients << 0.0, 0.0,
    (g1.transpose() * (A.inverse() * B * 4.0      * B.transpose() * A.inverse()) * g1 + 2.0 * g1.transpose() * (-A.inverse() * B * 4.0)      * g2 + g2.transpose() * 4.0 * g2),
            (g1.transpose() * (A.inverse() * B * 4.0 * Sa * B.transpose() * A.inverse()) * g1 + 2.0 * g1.transpose() * (-A.inverse() * B * 4.0 * Sa) * g2 + g2.transpose() * 4.0 * Sa * g2),
            (g1.transpose() * (A.inverse() * B *  Sa * Sa * B.transpose() * A.inverse()) * g1 + 2.0 * g1.transpose() * (-A.inverse() * B *  Sa * Sa) * g2 + g2.transpose() *  Sa * Sa * g2);

    Eigen::Vector3d p_lambda;
    p_lambda << 4.0,
            2.0 * S(0,0) + 2.0 * S(1,1),
            S(0,0) * S(1,1) - S(1,0) * S(0,1);

    poly_coefficients -= conv(p_lambda, p_lambda);

    const float lambda = (float) find_greatest_real_root(poly_coefficients.reverse());
    const Eigen::Vector4d x = -(M + 2.0 * lambda * W).inverse() * g;

    const Eigen::Vector2d translation = x.segment(0,2);
    const float theta = std::atan2(x(3),x(2));

    //std::cout << "translation: [" << translation(0) << ", " << translation(1) << "]" << std::endl;
    //std::cout << "rotation: " << (theta * 180.0f / M_PI) << std::endl;

    return(Transform(translation(0), translation(1), theta));
}