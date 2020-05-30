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
#include <json/eigen_conversion.hpp>

namespace motion_planner {

    template<typename T>
    using Path = std::deque<Eigen::Matrix<T, 2, 1> >;

    typedef Path<float> MapPath;
    typedef Path<int>   GridPath;

    static_assert((std::is_same<MapPath::value_type::value_type, float>::value),
                  "motion_planner::MapPath value type must be (float)");
    static_assert((std::is_same<GridPath::value_type::value_type, int>::value),
                  "motion_planner::GridPath value type must be (int)");

    MapPath TransformPath(const GridPath& grid_path, const Eigen::Affine3f& T_grid_to_map);
    GridPath TransformPath(const MapPath& map_path, const Eigen::Affine3f& T_map_to_grid);

    template<typename T, typename U>
    std::deque<Eigen::Matrix<T, 2, 1> > ConvertPath(const std::deque<Eigen::Matrix<U, 2, 1> > &input_path) {
        typename std::deque<Eigen::Matrix<T, 2, 1> > output_path;
        for (const Eigen::Matrix<U, 2, 1> &p : input_path) {
            output_path.push_back(p.template cast<T>());
        }
        return (output_path);
    }

} // namespace motion_planner

#define ConvertMapPathToGridPath(map_path, grid_path) \
        motion_planner::ConvertPath<int,float>(map_path, grid_path)

#define ConvertGridPathToMapPath(grid_path, map_path) \
        motion_planner::ConvertPath<float,int>(grid_path, map_path)

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
            path.push_back(Eigen::Vector2f(j_elem.at("x").get<T>(),
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