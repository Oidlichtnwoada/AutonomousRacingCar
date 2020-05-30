/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/path.h>

namespace motion_planner {

    MapPath TransformPath(const GridPath& grid_path, const Eigen::Affine3f& T_grid_to_map) {
        MapPath path;
        for(const Eigen::Vector2i& p : grid_path) {
            path.push_back(Eigen::Vector3f(T_grid_to_map * Eigen::Vector3f(p(0), p(1), 0.0)).head(2));
        }
        return(path);
    }

    GridPath TransformPath(const MapPath& path, const Eigen::Affine3f& T_map_to_grid) {
        GridPath grid_path;
        for(const Eigen::Vector2f& p : path) {
            grid_path.push_back(Eigen::Vector3f(T_map_to_grid * Eigen::Vector3f(p(0), p(1), 0.0)).cast<int>().head(2));
        }
        return(grid_path);
    }
} // namespace motion_planner
