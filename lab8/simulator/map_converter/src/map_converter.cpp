/*
 * ---------------------------------------------------------------------------------------------------------------------
 * Author: Thomas Pintaric (thomas.pintaric@gmail.com)
 * Based on prior work by: Joseph Auckley, Matthew O'Kelly, Aman Sinha, Hongrui Zheng
 * SPDX-License-Identifier: MIT
 * ---------------------------------------------------------------------------------------------------------------------
 */

#include <iostream>
#include <memory>

#if __GNUC__ >= 8
    #include <filesystem>
    namespace fs = std::filesystem;
#else
    #include <experimental/filesystem>
    namespace fs = std::experimental::filesystem;
#endif

#include <algorithm>
#include <tclap/CmdLine.h>
#include <yaml-cpp/yaml.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <octomap/octomap.h>

#define DEFAULT_INPUT_FILENAME "map.yaml"
#define DEFAULT_OUTPUT_FILENAME "map.ot"

int main(int argc, char** argv) {

    TCLAP::CmdLine cmd("Map-to-Octree Converter", ' ', "1.0");
    TCLAP::ValueArg<std::string> input_filename("i", "input", "input .yaml", true, DEFAULT_INPUT_FILENAME, "string");
    TCLAP::ValueArg<std::string> output_filename("o", "output", "output .octree", true, DEFAULT_OUTPUT_FILENAME, "string");

    try {
        cmd.add(input_filename);
        cmd.add(output_filename);
        cmd.parse(argc,argv);

        fs::path yaml_path = input_filename.getValue();
        if(!fs::exists(yaml_path)) {
            std::ostringstream o;
            o << "could not find " << yaml_path;
            throw std::runtime_error(o.str());
        }

        YAML::Node config = YAML::LoadFile(input_filename.getValue());

        const std::vector<std::string> required_keys { "image",
                                                       "resolution",
                                                       "origin",
                                                       "negate",
                                                       "occupied_thresh",
                                                       "free_thresh" };

        for(const std::string& key : required_keys) {
            if(!config[key].IsDefined()) {
                std::ostringstream o;
                o << "\"" << key << "\" is missing from \"" << input_filename.getValue() << "\"";
                throw std::runtime_error(o.str());
            }
        }

        const std::string image_filename = config["image"].as<std::string>();
        fs::path image_path = fs::path(yaml_path).replace_filename(fs::path(image_filename));
        if(!fs::exists(image_path)) {
            std::ostringstream o;
            o << "could not find " << image_path;
            throw std::runtime_error(o.str());
        }

        std::cout << "Reading image..." << std::endl;
        cv::Mat image = cv::imread(image_path.generic_string(), cv::IMREAD_GRAYSCALE);
        if(image.empty()) {
            std::ostringstream o;
            o << "could not read " << image_path;
            throw std::runtime_error(o.str());
        }

        const double resolution = config["resolution"].as<double>();
        const std::vector<double> origin = config["origin"].as<std::vector<double> >();
        if(origin.size() < 2) {
            throw std::runtime_error("invalid \"origin\"");
        }

        // Value interpretation:
        // (See: http://wiki.ros.org/map_server)
        //
        // If negate is false, p = (255 - x) / 255.0. This means that black (0) now has the highest value (1.0)
        // and white (255) has the lowest (0.0).
        //
        // If negate is true, p = x / 255.0. This is the non-standard interpretation of images, which is why it is
        // called negate, even though the math indicates that x is not negated. Nomenclature is hard. [sic!]

        const bool negate = (config["negate"].as<int>() != 0);
        if(!negate) {
            cv::bitwise_not(image, image);
        }

        const double occupied_thresh = config["occupied_thresh"].as<double>();
        const double free_thresh = config["free_thresh"].as<double>(); // will be ignored

        cv::Mat normalized_image;
        image.convertTo(normalized_image, CV_32FC1, 1.0f/255.0f); // convert to range [0,1]

        std::shared_ptr<octomap::OcTree> octree(new octomap::OcTree(resolution));
        unsigned int number_of_leaves = 0;

        for(size_t col = 0; col < normalized_image.cols; col++) {
            for(size_t row = 0; row < normalized_image.rows; row++) {
                const float cell_value = normalized_image.at<float>(row,col);
                const bool cell_is_occupied = cell_value >= (float) occupied_thresh;
                if(cell_is_occupied) {
                    /*
                     * NOTE: We use [x][y] == [col][row] here, i.e. column-major storage
                     */
                    const double x = (((double) col + 0.5) * resolution) + origin[0];
                    const double y = (((double) row + 0.5) * resolution) + origin[1];
                    const double z = (resolution / 2.0);
                    octree->updateNode(octomap::point3d(x, y, z), true, true);
                    number_of_leaves++;
                }
            }
        }
        octree->updateInnerOccupancy();
        {
            double upper_bound_x;
            double upper_bound_y;
            double upper_bound_z;
            double lower_bound_x;
            double lower_bound_y;
            double lower_bound_z;
            octree->getMetricMin(lower_bound_x, lower_bound_y, lower_bound_z);
            octree->getMetricMax(upper_bound_x, upper_bound_y, upper_bound_z);
            std::cout << "Octree bounds: " <<
                      "[" << lower_bound_x << ", " << lower_bound_y << "]" << " - " <<
                      "[" << upper_bound_x << ", " << upper_bound_y << "], " <<
                      number_of_leaves << " leaves inserted." << std::endl;
        }

        #if !defined(NDEBUG)

        // Validate octree

        for(size_t col = 0; col < normalized_image.cols; col++) {
            for(size_t row = 0; row < normalized_image.rows; row++) {
                const float cell_value = normalized_image.at<float>(row,col);
                const bool node_should_be_occupied = cell_value >= (float) occupied_thresh;
                const double x = (((double) col + 0.5) * resolution) + origin[0];
                const double y = (((double) row + 0.5) * resolution) + origin[1];
                const octomap::OcTreeNode* node = octree->search(x, y, 0.0);
                if(node_should_be_occupied && node == nullptr) {
                    throw std::runtime_error("octree validation failed");
                }
                if(node != nullptr) {
                    const bool node_is_occupied = octree->isNodeOccupied(node);
                    if(node_should_be_occupied != node_is_occupied) {
                        throw std::runtime_error("octree validation failed");
                    }
                }
            }
        }
        #endif

        fs::path octree_path = output_filename.getValue();

        if(!octree->writeBinary(octree_path.generic_string())) {
            std::ostringstream o;
            o << "could not write to " << octree_path;
            throw std::runtime_error(o.str());
        }


    } catch (TCLAP::ArgException &e) {
        std::cerr << "ERROR: " << e.error() << " for argument " << e.argId() << std::endl;
    } catch (YAML::RepresentationException &e) {
        std::cerr << "ERROR parsing \"" << input_filename.getValue() << "\": " << e.msg << std::endl;
    } catch (std::exception &e) {
        std::cerr << "ERROR: " << e.what() << std::endl;
    }
    return 0;
}
