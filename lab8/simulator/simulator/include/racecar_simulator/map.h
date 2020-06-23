/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Author: Thomas Pintaric (thomas.pintaric@gmail.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 * ---------------------------------------------------------------------------------------------------------------------
 */

#pragma once
#include <string>
#include <memory>
#include <octomap/octomap.h>
#include <boost/multi_array.hpp>
#include <Eigen/Core>

#define LUT_FILE_EXTENSION ".lidar_lut.gz"

namespace f1tenth {

    class Map {
    public:
        Map(std::string octree_filename,
            Eigen::Vector2f map_origin,
            float map_resolution,
            std::string drivable_grid_filename = "",
            bool enable_lut_support = false,
            unsigned int rays_per_lut_bin = 0);

        std::tuple<float,float,float,float> getMetricBounds() const;
        inline unsigned int getNumberOfNodes() const {
            return octree_->calcNumNodes();
        }

        inline std::shared_ptr<const octomap::OcTree> getOctree() const {
            return octree_;
        }

        inline bool supportsLookupTable() const {
            return can_use_lut_;
        }

        std::tuple<bool,float, Eigen::Vector2f>
        castRay(Eigen::Vector2f ray_center,
                Eigen::Vector2f ray_direction,
                float max_range) const noexcept; // returns <got_a_hit, ray_length, ray_endpoint>

        std::tuple<bool,float, Eigen::Vector2f>
        lookupRayLength(Eigen::Vector2f ray_center,
                        float ray_angle,
                        float max_range) const noexcept; // returns <got_a_hit, ray_length, ray_endpoint>


    protected:
        Eigen::Vector2i worldToGrid(Eigen::Vector2f world_coordinates) const;
        Eigen::Vector2f gridToWorld(Eigen::Vector2i grid_coordinates) const;

        bool loadLookupTableFromFile(std::string lut_filename);
        void computeLookupTable(float max_ray_length = 0.0f);
        bool saveLookupTableToFile(std::string lut_filename, bool overwrite_existing_file = false) const;

        std::shared_ptr<octomap::OcTree> octree_;

        const Eigen::Vector2f map_origin_;
        const float map_resolution_;

        const bool can_use_lut_;
        const unsigned int rays_per_lut_bin_; // same for each LUT entry
        static constexpr float lut_starting_angle_ = (-M_PI);

        typedef boost::multi_array<float*, 2> LookupTable;
        LookupTable lut_; // column-major storage. i.e. [x][y] == [col][row]
        std::vector<Eigen::Vector2i> nonzero_lut_bins_;
        std::shared_ptr<float[]> lut_data_;
        size_t number_of_allocated_lut_bins_; // size in bytes: number_of_allocated_lut_bins_ * sizeof(float)

    };

} // namespace f1tenth
