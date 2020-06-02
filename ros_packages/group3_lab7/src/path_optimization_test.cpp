/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <iostream>
#include <fstream>
#include <motion_planner/path.h>
#include <motion_planner/path_optimizer.h>
#include <motion_planner/occupancy_grid.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>


template <typename T>
T normal_pdf(T x, T mu, T sigma)
{
    constexpr T inv_sqrt_2pi = 0.3989422804014327;
    const T a = (x - mu) / sigma;
    return inv_sqrt_2pi / sigma * std::exp(-T(0.5) * a * a);
}

template <typename T>
T standard_normal_pdf(T x)
{
    constexpr T mu = 0.0;
    constexpr T sigma = 1.0;
    constexpr T inv_sqrt_2pi = 0.3989422804014327;
    const T a = (x - mu) / sigma;
    return inv_sqrt_2pi / sigma * std::exp(-T(0.5) * a * a);
}

template <typename T>
T zero_mean_normal_pdf(T x, T sigma)
{
    constexpr T mu = 0.0;
    constexpr T inv_sqrt_2pi = 0.3989422804014327;
    const T a = (x - mu) / sigma;
    return inv_sqrt_2pi / sigma * std::exp(-T(0.5) * a * a);
}




using namespace motion_planner;
using nlohmann::json;

template <typename T>
void PrintPath(std::string name, const Path<T>& path) {
    std::cout << name << ": ";
    for(int i=0; i<path.size(); i++) {
        if(i>0) {
            std::cout << ", ";
        }
        std::cout << "[" << path[i](0) << ", " << path[i](1) << "]";
    }
    std::cout << std::endl;
}

#define OPTIM_ENABLE_EIGEN_WRAPPERS
#include <optim.hpp>

using namespace optim;

struct OptimizationProblem {

    OptimizationProblem(Path<float> path, OccupancyGrid occupancy_grid)
            : path_(path)
            , normals_(ApproximateNormals<float>(path_))
            , occupancy_grid_(occupancy_grid)
            , path_length_(LinearPathLength<float>(path_))
    {
        assert(path_.size() >= 2);
        initial_distance_from_nearest_obstacle_.resize(path_.size());
        average_distance_from_nearest_obstacle_ = 0.0f;
        for(int i=0; i<path_.size(); i++) {
            const Eigen::Vector2f& waypoint = path_.at(i);
            const float distance_from_nearest_obstacle =
                    occupancy_grid_.interpolatedDistanceFromNearestObstacle(waypoint);
            initial_distance_from_nearest_obstacle_[i] = distance_from_nearest_obstacle;
            average_distance_from_nearest_obstacle_ += distance_from_nearest_obstacle;
        }
        average_distance_from_nearest_obstacle_ /= (float) path_.size();
    }

    Path<float> applyToPath(const optim::Vec_t& x) const {
        assert(path_.size() - 2 == x.size());
        Path<float> path(path_.size());
        path.front() = path_.front();
        for(int i=1; i < path_.size()-1; i++) {
            path[i] = path_[i] + normals_[i] * x[i-1];
        }
        path.back() = path_.back();
        return(path);
    }

    double cost(const optim::Vec_t& x) const {
        assert(x.size() == path_.size()-2);

        Path<float> modified_path = applyToPath(x);
        const float modified_path_length = LinearPathLength<float>(modified_path);

        const float avg_path_length_increase =
                (modified_path_length - path_length_) / (float) modified_path.size();

        float sum_of_weighted_distance_improvements = 0.0f;

        for(int i=0; i<modified_path.size(); i++) {

            if(modified_path[i](0) < 0 ||
               modified_path[i](0) >= occupancy_grid_.rows() ||
               modified_path[i](1) < 0 ||
               modified_path[i](1) >= occupancy_grid_.cols() ||
               occupancy_grid_.isGridCellOccupied(modified_path[i].cast<int>())) {

                //return std::numeric_limits<double>::max();
                return(1E6);
            }

            const float distance_from_nearest_obstacle =
                    occupancy_grid_.interpolatedDistanceFromNearestObstacle(modified_path[i]);

            float distance_improvement =
                    (distance_from_nearest_obstacle - initial_distance_from_nearest_obstacle_[i]);

            if(distance_improvement < 0.0f) {
                // assign penalty
                constexpr float kappa = 1.0f; // <--- parameter!
                sum_of_weighted_distance_improvements += distance_improvement * kappa;

            } else {

                constexpr float tau = 0.5f; // <--- parameter!
                const float weighted_distance_improvement =
                        (2.0f * (1.0f / (1.0f + std::exp(-distance_improvement * tau))) - 1.0f);
                sum_of_weighted_distance_improvements += weighted_distance_improvement;
            }
        }

        constexpr float rho = 0.5f; // <--- parameter!
        const float cost = (-sum_of_weighted_distance_improvements / (float) modified_path.size()) +
                           (avg_path_length_increase * rho);

        //const float cost = (-sum_of_weighted_distance_improvements);
        return(cost);
    }

    Path<float> path_;
    Path<float> normals_;
    OccupancyGrid occupancy_grid_;
    float path_length_;
    float average_distance_from_nearest_obstacle_;
    std::vector<float> initial_distance_from_nearest_obstacle_;
};

double optimization_fn(const Vec_t& vals_inp, Vec_t* grad_out, void* opt_data)
{
    assert(opt_data != nullptr);
    const OptimizationProblem &problem = *((OptimizationProblem*) opt_data);
    double cost = (double) problem.cost(vals_inp);

    // numerical gradient computation
    if(grad_out != nullptr) {
        const float epsilon = 0.5;
        Vec_t v = vals_inp;
        for(int j=0; j<vals_inp.size(); j++) {
            v[j] = vals_inp[j] + epsilon;
            const float cost_plus_epsilon = problem.cost(v);
            v[j] = vals_inp[j] - epsilon;
            const float cost_minus_epsilon = problem.cost(v);
            v[j] = vals_inp[j];
            (*grad_out)[j] = (cost_plus_epsilon - cost_minus_epsilon) / (2.0 * epsilon);
        }
    }
    return(cost);
}


class Optimizer {
public:
    Optimizer() {}

};

int main(int argc, char **argv) {

    GridPath grid_path;
    {
        std::ifstream i("grid_path.json");
        json j;
        i >> j;
        grid_path = PathFromJson<int>(j);
    }
    //PrintPath<int>("Grid path",grid_path);

    Path<float> path = ConvertPath<int,float,1>(grid_path);
    const float path_length = LinearPathLength<float>(path);
    const int number_of_points_on_resampled_path = 20;
    Path<float> interpolated_path = UniformLinearInterpolation<float>(
            path, number_of_points_on_resampled_path, path_length);

    PrintPath<float>("Path", path);
    PrintPath<float>("Interpolated path", interpolated_path);

    cv::Mat grid_image = cv::imread("occupancy_grid.png", cv::IMREAD_GRAYSCALE);
    OccupancyGrid occupancy_grid(grid_image, OccupancyGrid::L2_DISTANCE_METRIC_3x3_KERNEL);
    occupancy_grid.computeDistanceTransform();

    OptimizationProblem problem(interpolated_path, occupancy_grid);
    optim::Vec_t x = optim::Vec_t::Zero(interpolated_path.size()-2);

    optim::algo_settings_t settings;
    settings.vals_bound = true;
    settings.lower_bounds = optim::Vec_t::Ones(interpolated_path.size()-2) * (-5.0f);
    settings.upper_bounds = optim::Vec_t::Ones(interpolated_path.size()-2) * (+5.0f);

    //settings.de_initial_lb = optim::Vec_t::Ones(interpolated_path.size()-2) * (-2.0f);
    //settings.de_initial_ub = optim::Vec_t::Ones(interpolated_path.size()-2) * (+2.0f);

    settings.gd_method = 6;
    settings.gd_settings.step_size = 0.5;
    settings.iter_max = 100;

    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();

    bool success = optim::gd(x, optimization_fn, &problem, settings);

    std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end-start;
    std::cout << "Elapsed time: " << elapsed_seconds.count() << " seconds" << std::endl;

    //bool success = optim::de(x, optimization_fn, &problem, settings);

    //bool success = optim::nm(x, optimization_fn, &problem);

    const Path<float> optimized_path = problem.applyToPath(x);
    PrintPath<float>("Optimized path", optimized_path);


    cv::Mat grid_rgb_8u;
    cv::cvtColor(occupancy_grid.occupancy(), grid_rgb_8u, cv::COLOR_GRAY2RGB);

    cv::Mat grid_rgb_32f;
    grid_rgb_8u.convertTo(grid_rgb_32f, CV_32FC3);


    cv::Mat dist_32f;
    occupancy_grid.distances().convertTo(dist_32f, CV_32FC1);

    cv::Mat dist_mask;

    const cv::Scalar lower_distance(0);
    const cv::Scalar upper_distance(30);
    //const cv::Scalar upper_distance(255);

    cv::inRange(dist_32f, lower_distance, upper_distance, dist_mask);
    cv::Mat masked_dist_32f;
    dist_32f.copyTo(masked_dist_32f, dist_mask); // Your final mat with value between 300 and 400

    cv::inRange(dist_32f, upper_distance, cv::Scalar(255), dist_mask);
    masked_dist_32f.setTo(upper_distance, dist_mask);

    //cv::normalize(dist_32f,  dist_32f, 0, 1, cv::NORM_MINMAX);
    cv::normalize(masked_dist_32f,  masked_dist_32f, 0, 1, cv::NORM_MINMAX);

    cv::Mat dist_rgb_32f;
    cv::cvtColor(masked_dist_32f, dist_rgb_32f, cv::COLOR_GRAY2RGB);

    //cv::Mat montage_rgb_32f;
    //cv::hconcat(grid_rgb_32f, dist_rgb_32f, montage_rgb_32f);


    //# Draw a diagonal blue line with thickness of 5 px
    //cv.line(img,(0,0),(511,511),(255,0,0),5)

    cv::Mat display_img_rgb_8u;
    dist_rgb_32f *= 255.0;
    dist_rgb_32f.convertTo(display_img_rgb_8u, CV_8UC3);
    cv::applyColorMap(display_img_rgb_8u, display_img_rgb_8u, cv::COLORMAP_COOL);

    Path<int> _path;
    _path = ConvertPath<float,int,-1>(path);
    for(int i=1; i<_path.size(); i++) {
        const int x0 = _path[i-1](1);
        const int y0 = _path[i-1](0);
        const int x1 = _path[i](1);
        const int y1 = _path[i](0);
        //cv::line(grid_rgb_8u, cv::Point(x0,y0), cv::Point(x1,y1), cv::Scalar(255,0,0));
        //cv::Scalar colorScalar = cv::Scalar( 94, 206, 165 );
        cv::Scalar colorScalar = cv::Scalar( 0, 255, 255 );
        cv::line(display_img_rgb_8u, cv::Point(x0,y0), cv::Point(x1,y1), colorScalar);
    }

    _path = ConvertPath<float,int,-1>(optimized_path);
    for(int i=1; i<_path.size(); i++) {
        const int x0 = _path[i-1](1);
        const int y0 = _path[i-1](0);
        const int x1 = _path[i](1);
        const int y1 = _path[i](0);
        //cv::line(grid_rgb_8u, cv::Point(x0,y0), cv::Point(x1,y1), cv::Scalar(255,0,0));
        //cv::Scalar colorScalar = cv::Scalar( 94, 206, 165 );
        cv::Scalar colorScalar = cv::Scalar( 0, 0, 255 );
        cv::line(display_img_rgb_8u, cv::Point(x0,y0), cv::Point(x1,y1), colorScalar);
    }



    cv::namedWindow( "Display window", cv::WINDOW_KEEPRATIO );
    cv::imshow( "Display window", display_img_rgb_8u);
    cv::waitKey(0);
}