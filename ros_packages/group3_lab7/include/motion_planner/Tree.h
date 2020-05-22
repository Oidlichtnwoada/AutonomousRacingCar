/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#pragma once
#include <Eigen/Dense>
#include <nanoflann/nanoflann.hpp>
#include <random>

namespace motion_planner {

    template<typename T>
    struct BaseDistribution {
        typedef std::uniform_real_distribution<> UniformDistribution;
    };
    template<>
    struct BaseDistribution<int> { // partial specialization
        typedef std::uniform_int_distribution<> UniformDistribution;
    };

    template<typename T>
    struct BaseTree : public BaseDistribution<T> {
        typedef nanoflann::KNNResultSet<T> KNNResultSet;
        typedef Eigen::Matrix<T, 2, 1> Vector2;

        struct Node {

            Node(Vector2 position, size_t parent, T path_length = static_cast<T>(0))
                : position_(position)
                , parent_(parent)
                , path_length_(path_length)
                {}

            Node(T x, T y, size_t parent, T path_length = static_cast<T>(0))
                : position_(Vector2(x, y))
                , parent_(parent)
                , path_length_(path_length)
                {}

            Vector2 position_;
            size_t parent_;
            T path_length_; // accumulated path length from the root to this node,
                            // a.k.a. "cost", only relevant for RRT*
        };

        std::vector<Node> nodes_;
        inline size_t kdtree_get_point_count() const { return nodes_.size(); }
        inline T kdtree_get_pt(const size_t idx, const size_t dim) const { return nodes_[idx].position_(dim); }
        template<class BBOX>
        bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
    };

    template<typename T, bool USE_L1_DISTANCE_METRIC>
    struct Tree : public BaseTree<T> // default: L1 distance metric
    {
        typedef std::uniform_int_distribution<> Distribution;
        typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L1_Adaptor<T, Tree<T, USE_L1_DISTANCE_METRIC> >, Tree<T, USE_L1_DISTANCE_METRIC>, 2 /* dimensions */> KDTree;
    };

    template<typename T>
    struct Tree<T, false> : public BaseTree<T> { // partial specialization for L2 distance metric
        typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<nanoflann::L2_Simple_Adaptor<T, Tree<T, false> >, Tree<T, false>, 2 /* dimensions */> KDTree;
    };

} // namespace motion_planner