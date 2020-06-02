/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <deque>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <json/json.hpp>

namespace motion_planner {

    template<typename T>
    using Path = std::deque<Eigen::Matrix<T, 2, 1> >;

    typedef Path<float> MapPath;
    typedef Path<int>   GridPath;

    static_assert((std::is_same<MapPath::value_type::value_type, float>::value),
                  "motion_planner::MapPath value type must be (float)");
    static_assert((std::is_same<GridPath::value_type::value_type, int>::value),
                  "motion_planner::GridPath value type must be (int)");

    template<typename T, typename U>
    Path<U> TransformPath(const Path<T> from_path, const Eigen::Affine3f& transform) {
        Path<U> to_path;
        for(const auto& p : from_path) {
            to_path.push_back(Eigen::Vector3f(transform * Eigen::Vector3f(p(0), p(1), 0.0)).template cast<U>().head(2));
        }
        return(to_path);
    }

    //MapPath TransformPath(const GridPath& grid_path, const Eigen::Affine3f& T_grid_to_map);
    //GridPath TransformPath(const MapPath& map_path, const Eigen::Affine3f& T_map_to_grid);

    template<typename U, typename T, int pixel_center_offset = 0>
    std::deque<Eigen::Matrix<T, 2, 1> > ConvertPath(const std::deque<Eigen::Matrix<U, 2, 1> > &input_path) {
        typename std::deque<Eigen::Matrix<T, 2, 1> > output_path;
        for (const Eigen::Matrix<U, 2, 1> &p : input_path) {
            output_path.push_back(p.template cast<T>());
            if(pixel_center_offset>0) {
                output_path.back() += Eigen::Matrix<T, 2, 1>(0.5, 0.5);
            } else if(pixel_center_offset<0) {
                output_path.back() -= Eigen::Matrix<T, 2, 1>(0.5, 0.5);
            }
        }
        return (output_path);
    }

    template<typename T>
    Path<float> ApproximateNormals(const Path<T>& path, int window_width=3) {
        assert(path.size() >= window_width);
        Path<float> normals;
        normals.resize(path.size());
        const int half_window_width = std::max<int>(1, (window_width-1)/2);
        for(int j=0; j<path.size(); j++) {
            const int i = std::max<int>(0,j-half_window_width);
            const int k = std::min<int>(path.size()-1,j+half_window_width);
            Eigen::MatrixXf M(k-i+1, 2);
            for(int m=0; m<=(k-i); m++) {
                M.row(m) = path[i+m].template cast<float>();
            }
            const Eigen::MatrixXf centered = M.rowwise() - M.colwise().mean();
            const Eigen::MatrixXf covariance = centered.adjoint() * centered;
            const Eigen::Vector2f normal = Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf>(
                    covariance).eigenvectors().leftCols(1);
            normals[j] = (normal.template cast<float>()).normalized();
        }
        return(normals);
    }

} // namespace motion_planner

#define ConvertMapPathToGridPath(map_path, grid_path) \
        motion_planner::ConvertPath<float,int>(map_path, grid_path)

#define ConvertGridPathToMapPath(grid_path, map_path) \
        motion_planner::ConvertPath<int,float>(grid_path, map_path)

namespace motion_planner {

    template<typename T>
    nlohmann::json PathToJson(const Path<T>& path) {
        nlohmann::json j = nlohmann::json::array();
        for(const auto& point : path) {
            j.push_back(nlohmann::json {{"x",point(0)}, {"y",point(1)} });
        }
        return(j);
    }

    template<typename T>
    Path<T> PathFromJson(const nlohmann::json& j) {
        Path<T> path;
        for(const auto& j_elem : j) {
            path.push_back(typename Path<T>::value_type(j_elem.at("x").get<T>(),
                                                        j_elem.at("y").get<T>()));
        }
        return(path);
    }
} // namespace motion_planner

namespace motion_planner {

    template<typename T>
    T LinearPathLength(std::deque<Eigen::Matrix<T, 2, 1> > const &path) {
        T sum = 0.0;
        for(unsigned int j=1; j < path.size(); j++) {
            sum += (path[j]-path[j-1]).norm();
        }
        return(sum);
    }

    template<typename T>
    std::deque<Eigen::Matrix<T, 2, 1> > UniformLinearInterpolation(
            const std::deque<Eigen::Matrix<T, 2, 1> > &path,
            std::size_t target_size,
            const T known_linear_path_length = 0) {

        std::deque<Eigen::Matrix<T, 2, 1> > interpolated_path;
        if(path.size() < 2 || target_size < 2) {
            // degenerate input path or invalid target size
            return interpolated_path;
        }
        const T total_length = (known_linear_path_length > 0.0f) ?
                               known_linear_path_length : LinearPathLength<T>(path);

        const T segment_length = total_length / (T)(target_size - 1);

        auto start = 0;
        auto finish = start + 1;
        T src_segment_offset = 0.0f;
        T src_segment_length =  (path[start] - path[finish]).norm();

        interpolated_path.push_back(path[start].template cast<T>());

        for(size_t i=1; i<target_size-1; i++) {
            const T next_offset = segment_length * i;
            while(src_segment_offset + src_segment_length < next_offset) {
                src_segment_offset += src_segment_length;
                start = finish++;
                src_segment_length = (path[start] - path[finish]).norm();
            }
            const T part_offset = next_offset - src_segment_offset;
            const T part_ratio = part_offset / src_segment_length;
            interpolated_path.push_back(Eigen::Matrix<T, 2, 1>(
                    path[start](0) +  part_ratio * (path[finish](0) - path[start](0)),
                    path[start](1) +  part_ratio * (path[finish](1) - path[start](1))));
        }
        interpolated_path.push_back(path.back().template cast<T>());
        return interpolated_path;
    }
} // namespace motion_planner