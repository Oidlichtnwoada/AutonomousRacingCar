/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Author: Thomas Pintaric (thomas.pintaric@gmail.com)
 * SPDX-License-Identifier: GPL-3.0-or-later
 * ---------------------------------------------------------------------------------------------------------------------
 */

#include <racecar_simulator/map.h>
#include <cstring>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#if __GNUC__ >= 8
#include <filesystem>
    namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#include <iostream>
#include <iomanip>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace f1tenth {

    Map::Map(std::string octree_filename,
             Eigen::Vector2f map_origin,
             float map_resolution,
             std::string drivable_grid_filename,
             bool enable_lut_support,
             unsigned int rays_per_lut_bin)

        : octree_(new octomap::OcTree(octree_filename))
        , map_origin_(map_origin)
        , map_resolution_(map_resolution)
        , can_use_lut_(enable_lut_support)
        , rays_per_lut_bin_(rays_per_lut_bin)
        {
            octree_->prune();

            const auto [lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y] = getMetricBounds();
            std::cout << "Loaded map from \"" << octree_filename << "\". Octree bounds: " <<
                      "[" << lower_bound_x << ", " << lower_bound_y << "]" << " - " <<
                      "[" << upper_bound_x << ", " << upper_bound_y << "], " <<
                      getNumberOfNodes() << " nodes." << std::endl;

            if(not can_use_lut_) {
                return;
            }

            cv::Mat drivable_grid = cv::imread(drivable_grid_filename, cv::IMREAD_GRAYSCALE);
            if(drivable_grid.empty()) {
                std::ostringstream o;
                o << "could not open " << fs::path(drivable_grid_filename).relative_path();
                throw std::runtime_error(o.str());
            }

            const int number_of_drivable_grid_cells = cv::countNonZero(drivable_grid);
            const int& rows = drivable_grid.rows;
            const int& cols = drivable_grid.cols;
            lut_.resize(boost::extents[rows][cols]);
            std::uninitialized_fill_n(lut_.data(), lut_.num_elements(), nullptr);

            number_of_allocated_lut_bins_ = number_of_drivable_grid_cells * rays_per_lut_bin_;
            try {
                lut_data_ = std::shared_ptr<float[]>(new float[number_of_allocated_lut_bins_]);
                assert(lut_data_.get() != nullptr);
                std::memset((void*) lut_data_.get(), 0, number_of_allocated_lut_bins_ * sizeof(float));
            } catch(std::exception& e) {
                std::ostringstream o;
                o << "unable to allocate " << (number_of_allocated_lut_bins_ * sizeof(float) / (1024 * 1024)) << "MB of LUT memory";
                throw std::runtime_error(o.str());
            }

            cv::Mat nonzero_coordinates;
            cv::findNonZero(drivable_grid, nonzero_coordinates);
            float* lut_bin_ptr = lut_data_.get();
            for(int i = 0; i < nonzero_coordinates.total(); i++ ) {
                const cv::Point& point = nonzero_coordinates.at<cv::Point>(i);
                nonzero_lut_bins_.push_back(Eigen::Vector2i(point.x, point.y));
                lut_[point.x /* col */][point.y /* row */] = lut_bin_ptr; // NOTE: column-major storage
                assert(lut_bin_ptr <= lut_data_.get() + number_of_allocated_lut_bins_);
                lut_bin_ptr += rays_per_lut_bin_;
            }
            assert(lut_bin_ptr == lut_data_.get() + number_of_allocated_lut_bins_);


            const fs::path lut_path = fs::path(drivable_grid_filename).replace_extension(LUT_FILE_EXTENSION);

            {
                fs::path stem = lut_path.filename();
                while(stem.has_extension()) {
                    stem = stem.stem();
                }
                std::cout << "Allocated " << (number_of_allocated_lut_bins_ * sizeof(float) / (1024 * 1024)) <<
                          "MB of lookup table memory for " << stem.relative_path() << "." << std::endl;
            }

            if(not loadLookupTableFromFile(lut_path)) {
                computeLookupTable();
                if(not saveLookupTableToFile(lut_path, true)) {
                    std::ostringstream o;
                    o << "unable to write lookup table to " << lut_path.relative_path() << ".";
                    throw std::runtime_error(o.str());
                }
            }
        }

    std::tuple<bool,float, Eigen::Vector2f>
    Map::castRay(Eigen::Vector2f ray_center,
                 Eigen::Vector2f ray_direction,
                 float max_range) const noexcept {

        assert(max_range > 0.0f);

        octomap::point3d end;
        const bool got_a_hit = octree_->castRay(octomap::point3d(ray_center.x(), ray_center.y(), 0.0),
                                                octomap::point3d(ray_direction.x(), ray_direction.y(), 0.0),
                                                end,
                                                true, // treat unknown grid cells as free
                                                max_range);

        if(got_a_hit) {
            const Eigen::Vector2f ray_endpoint = Eigen::Vector2f(end.x(), end.y());
            const float ray_length = (ray_endpoint - ray_center).norm();
            return {true, ray_length, ray_endpoint};
        } else {
            const Eigen::Vector2f ray_endpoint = Eigen::Vector2f(ray_center.x(), ray_center.y()) + \
                Eigen::Vector2f(ray_direction.x(), ray_direction.y()).normalized() * max_range;
            return {false, max_range, ray_endpoint};
        }
    }

    std::tuple<bool,float, Eigen::Vector2f>
    Map::lookupRayLength(Eigen::Vector2f ray_center,
                         float ray_angle,
                         float max_range) const noexcept {

        const Eigen::Vector2i grid_coordinate = worldToGrid(ray_center);
        const float* lut_bin = lut_[grid_coordinate.x()][grid_coordinate.y()];
        if(lut_bin == nullptr) {
            return {false, std::numeric_limits<float>::quiet_NaN(), ray_center.cast<float>()};
        }

        const int ray_index = static_cast<int>(((ray_angle - lut_starting_angle_) / (2.0f * M_PI)) *
                                               (float) rays_per_lut_bin_) % rays_per_lut_bin_;

        float ray_length = lut_bin[ray_index];
        if(std::isnan(ray_length)) {
            ray_length = max_range;
        }

        const Eigen::Vector2f normalized_ray_direction =
                Eigen::Vector2f(Eigen::Rotation2Df(ray_angle) * Eigen::Vector2f(1.0f, 0.0f));

        const Eigen::Vector2f ray_endpoint = ray_center.cast<float>() + normalized_ray_direction * ray_length;

        #ifdef VERIFY_LOOKUP_TABLE
        {
            const Eigen::Vector2f grid_aligned_ray_center = gridToWorld(grid_coordinate);
            const auto [_got_a_hit, _ray_length, _ray_endpoint] =
                    castRay(grid_aligned_ray_center, normalized_ray_direction, max_range);
            const float error = std::abs(ray_length - _ray_length);
            constexpr float max_error = 0.25f; // TODO: this needs to be computed based on the LUT discretization
            assert(error < max_error);
        }
        #endif
        return {true, ray_length, ray_endpoint};
    }

    bool Map::loadLookupTableFromFile(std::string lut_filename) {
        assert(can_use_lut_);
        assert(lut_data_);
        const fs::path lut_path(lut_filename);
        if(not fs::exists(lut_path)) {
            return(false);
        }

        boost::iostreams::array_sink output(
                (char*) lut_data_.get(),
                (std::streamsize) number_of_allocated_lut_bins_ * sizeof(float));

        boost::iostreams::filtering_istreambuf input;
        input.push(boost::iostreams::gzip_decompressor());
        input.push(boost::iostreams::file_source(lut_path, std::ios_base::in | std::ios_base::binary));

        const std::streamsize bytes_copies = boost::iostreams::copy(input, output);

        if(bytes_copies != (std::streamsize) number_of_allocated_lut_bins_ * sizeof(float)) {
            std::ostringstream o;
            o << "error loading pre-computed lookup table from " << lut_path.relative_path();
            throw std::runtime_error(o.str());
        }

        std::cout << "Loaded lookup table (" << (bytes_copies/(1024*1024)) << "MB) from " <<
                  lut_path.relative_path() << std::endl;
        return true;
    }

    void Map::computeLookupTable(float max_ray_length) {
        assert(can_use_lut_);
        assert(lut_data_);

        if(max_ray_length <= 0.0f) {
            const auto [lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y] = getMetricBounds();
            max_ray_length = std::ceil((Eigen::Vector2f(upper_bound_x, upper_bound_y) -
                                        Eigen::Vector2f(lower_bound_x, lower_bound_y)).norm());
        }

        std::cout << "Computing lookup table (" << rays_per_lut_bin_ << " rays per bin, "
                  << nonzero_lut_bins_.size() << " bins, " << max_ray_length <<
                  "m max. ray length)..." << std::endl;

        const float angular_increment = (2.0f * M_PI) / (float) (rays_per_lut_bin_);

        std::atomic<int> number_of_bins_computed(0);

        #ifndef DO_NOT_USE_OPENMP
        #pragma omp parallel for
        #endif
        for(int i=0; i < nonzero_lut_bins_.size(); i++) {

            const int& grid_x = nonzero_lut_bins_[i].x();
            const int& grid_y = nonzero_lut_bins_[i].y();
            const Eigen::Vector2f ray_center = gridToWorld(Eigen::Vector2i(grid_x, grid_y));

            float* rays = lut_[grid_x][grid_y];
            assert(rays != nullptr);
            for(int j=0; j < rays_per_lut_bin_; j++) {
                const float ray_angle = lut_starting_angle_ + (float) j * angular_increment;
                const auto [got_a_hit, ray_length, _] = castRay(ray_center,
                                                                Eigen::Vector2f(Eigen::Rotation2Df(ray_angle) *
                                                                                Eigen::Vector2f(1.0f, 0.0f)),
                                                                max_ray_length);
                rays[j] = got_a_hit ? ray_length : std::numeric_limits<float>::quiet_NaN();
                if(!got_a_hit) {
                    // This is likely a result of an invalid/outdated drivable area bitmap.
                    throw std::runtime_error("unexpected ray casting miss");
                }
            }
            const int n = number_of_bins_computed.fetch_add(1);
            if((n+1) % 1000 == 0 || (n+1) == nonzero_lut_bins_.size()) {
                std::cout << (n+1) << " of " << nonzero_lut_bins_.size() << " LUT bins computed (" <<
                          ((100*(n+1)) / nonzero_lut_bins_.size()) << "% complete)" << std::endl;
            }
        }
        std::cout << "Lookup table consists of " << nonzero_lut_bins_.size() <<
                  " non-empty bins." << std::endl;
    }

    bool Map::saveLookupTableToFile(std::string lut_filename, bool overwrite_existing_file) const {
        assert(can_use_lut_);
        assert(lut_data_);
        const fs::path lut_path(lut_filename);
        if(fs::exists(lut_path) and (not overwrite_existing_file)) {
            return(false);
        }

        boost::iostreams::array_source input(
                (char*) lut_data_.get(),
                (std::streamsize) number_of_allocated_lut_bins_ * sizeof(float));

        boost::iostreams::filtering_ostreambuf output;
        output.push(boost::iostreams::gzip_compressor());
        output.push(boost::iostreams::file_sink(lut_path, std::ios_base::out | std::ios_base::binary));

        const std::streamsize bytes_copies = boost::iostreams::copy(input, output);

        if(bytes_copies != (std::streamsize) number_of_allocated_lut_bins_ * sizeof(float)) {
            std::ostringstream o;
            o << "error writing pre-computed lookup table to " << lut_path.relative_path();
            throw std::runtime_error(o.str());
        }

        std::cout << "Saved lookup table (" << (bytes_copies/(1024*1024)) << "MB) to " <<
                  lut_path.relative_path() << std::endl;
        return(true);
    }

    Eigen::Vector2i Map::worldToGrid(Eigen::Vector2f world_coordinates) const {
        return ((world_coordinates - map_origin_) / map_resolution_).cast<int>();
    }
    Eigen::Vector2f Map::gridToWorld(Eigen::Vector2i grid_coordinates) const {
        return (((grid_coordinates.cast<float>() + Eigen::Vector2f(0.5f, 0.5f)) * map_resolution_) + map_origin_);
    }

    std::tuple<float,float,float,float> Map::getMetricBounds() const {
        double lower_bound_x;
        double lower_bound_y;
        double lower_bound_z;
        double upper_bound_x;
        double upper_bound_y;
        double upper_bound_z;
        octree_->getMetricMin(lower_bound_x, lower_bound_y, lower_bound_z);
        octree_->getMetricMax(upper_bound_x, upper_bound_y, upper_bound_z);
        return {lower_bound_x, lower_bound_y, upper_bound_x, upper_bound_y};
    }

} // namespace f1tenth
