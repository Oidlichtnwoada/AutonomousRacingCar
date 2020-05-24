/* -------------------------------------------------------
 * VU Autonomous Racing Cars (2020S) - TU Wien
 * -------------------------------------------------------
 * Team 3: Adelmann, Brantner, Lukitsch, Pintaric
 * -------------------------------------------------------
 */

#include <motion_planner/path_planner.h>
#include <motion_planner/occupancy_grid.h>

PathPlanner::PathPlanner()
    : random_generator_(std::mt19937(PathPlanner::random_seed_))
{}

inline int L1_norm(int row0, int col0, int row1, int col1) {
    return Eigen::Vector2i(row0 - row1, col0 - col1).lpNorm<1>();
};
inline int L1_norm(const Eigen::Vector2i& v0, const Eigen::Vector2i& v1) {
    return Eigen::Vector2i(v1-v0).lpNorm<1>();
};
/*
inline int L1_norm(const Eigen::Vector2i& v0, int row1, int col1) {
    return Eigen::Vector2i(v0(0) - row1, v0(1) - col1).lpNorm<1>();
};
*/

std::tuple<bool, PathPlanner::Tree, PathPlanner::Path, PathPlanner::GridPath>
PathPlanner::run(Eigen::Vector2f goal_in_map_frame,
                 const OccupancyGrid& occupancy_grid,
                 const cv::Vec2i& occupancy_grid_center,
                 const Eigen::Affine3f& T_grid_to_map,
                 const Options options,
                 boost::shared_ptr<ros::Publisher> marker_publisher) {

    const bool USE_RRT_STAR = (options.algorithm == RRT_STAR) || (options.algorithm == INFORMED_RRT_STAR);
    const bool USE_INFORMED_RRT_STAR = (options.algorithm == INFORMED_RRT_STAR);

#ifndef NDEBUG
    unsigned int number_of_relinked_nodes = 0; // for debugging only
#endif

    constexpr size_t kdtree_max_leaf_size = 10; // TODO: Is this a good choice?

#if !defined(NDEBUG)
    constexpr unsigned int maximum_rrt_samples = 2000;  // TODO: expose this as a ROS node parameter
#else
    constexpr unsigned int maximum_rrt_samples = 4000;  // TODO: expose this as a ROS node parameter
#endif

    constexpr int maximum_rrt_expansion_distance = 5; // TODO: expose this as a ROS node parameter
    constexpr float rrt_goal_proximity_threshold = 2;   // TODO: expose this as a ROS node parameter

    // RRT* only:
    constexpr unsigned int rrt_star_nn_k = 10; // size of the neighborhood for reconnection inspection

    assert(maximum_rrt_samples > 0);
    assert(maximum_rrt_expansion_distance > 1);
    assert(rrt_goal_proximity_threshold > 0);

    Tree tree;
    KDTree index(2, tree, nanoflann::KDTreeSingleIndexAdaptorParams(kdtree_max_leaf_size));

    bool path_to_goal_found = false;
    T length_of_best_path_to_goal = std::numeric_limits<T>::max();

    const Eigen::Vector3f goal_in_grid_frame =
            T_grid_to_map.inverse() *
            Eigen::Vector3f(goal_in_map_frame(0), goal_in_map_frame(1), 0.0f);

    const T goal_row = static_cast<T>(goal_in_grid_frame(0));
    const T goal_col = static_cast<T>(goal_in_grid_frame(1));

    // check if goal lies outside grid bounds
    if(goal_row < 0 ||
       goal_row >= occupancy_grid.rows ||
       goal_col < 0 ||
       goal_col >= occupancy_grid.cols) {

        // goal lies outside grid bounds

        // TODO: Is there a better way to handle such a case?
        Path path;
        GridPath grid_path;
        return {false, tree, path, grid_path};
    }

    // Generate a uniform distribution across the entire grid (default for RRT, RRT* and
    // Informed-RRT* prior to finding a path to the goal).
    UniformDistribution uniform_row_distribution(static_cast<T>(0), static_cast<T>(occupancy_grid.rows - 1));
    UniformDistribution uniform_col_distribution(static_cast<T>(0), static_cast<T>(occupancy_grid.cols - 1));

    // Root node is the origin in the laser frame (= center of the dynamic occupancy grid)
    const T root_row = occupancy_grid_center(0);
    const T root_col = occupancy_grid_center(1);

    // For Informed-RRT* only:
    // Generate a uniform distribution across the best-fit (**) bounding rectangle (*) around the
    // ellipsoidal heuristic sampling domain described in [Gammell et al., 2014].
    //
    // (*)  this is our original contribution
    // (**) in the hyperellipsoid-aligned frame

    int heuristic_sampling_domain_major_axis_length = 0; // unknown at this point (for Informed-RRT* only)
    int heuristic_sampling_domain_minor_axis_length = 0; // unknown at this point (for Informed-RRT* only)
    UniformDistribution distribution_along_major_axis; // unknown at this point (for Informed-RRT* only)
    UniformDistribution distribution_along_minor_axis; // unknown at this point (for Informed-RRT* only)

    const Eigen::Vector2f root_to_goal(goal_row - root_row, goal_col - root_col); // can be pre-computed

    // For Informed-RRT* only:
    // Compute the sampling domain transformation ("hyperellipsoid-aligned frame" --> "grid frame"),
    // consisting of a rotation by [theta] and a translation by [root_row, root_row].
    const float theta = std::atan2(root_to_goal(1), root_to_goal(0));
    Eigen::Affine3f T_sampling_domain_to_grid_frame = Eigen::Affine3f::Identity(); // unknown at this point (for Informed-RRT* only)

    // For Informed-RRT* only:
    // The following lambda-function returns a valid(*) sample from the heuristic sampling domain.
    // (*) guaranteed to lie inside the bounds of the grid frame
    auto sample_from_heuristic_sampling_domain = [&]() -> std::tuple<int,int> {
        Eigen::Vector3f sample_in_grid_frame;
        do {
            // (x,y) are coordinates in the "hyperellipsoid-aligned frame"
            const T x = distribution_along_major_axis(random_generator_);
            const T y = distribution_along_minor_axis(random_generator_);
            // transform coordinates from "hyperellipsoid-aligned frame" to grid frame
            sample_in_grid_frame = T_sampling_domain_to_grid_frame * Eigen::Vector3f(x, y, 0.0f);

        } while(sample_in_grid_frame(0) < 0 ||
                sample_in_grid_frame(0) >= occupancy_grid.rows ||
                sample_in_grid_frame(1) < 0 ||
                sample_in_grid_frame(1) >= occupancy_grid.cols);

        return {static_cast<T>(sample_in_grid_frame(0)),
                static_cast<T>(sample_in_grid_frame(1))};
    };

    auto sample_from_entire_grid = [&]() -> std::tuple<int,int> {
        return { uniform_row_distribution(random_generator_),
                 uniform_col_distribution(random_generator_) };
    };

    const float shortest_linear_path_to_goal = root_to_goal.norm(); // L2 norm (this is also the lower bound of the L1 norm)
    //const float upper_bound_L1_distance_to_goal = shortest_linear_path_to_goal * std::sqrt(2.0); // upper bound for the L1 norm
    //const float lower_bound_L1_distance_to_goal = shortest_linear_path_to_goal; // lower bound for the L1 norm (same as the L2 norm)

    tree.nodes_.push_back(Node(root_row,root_col,0)); // insert the root node
    index.addPoints(tree.nodes_.size() - 1, tree.nodes_.size() - 1); // update kd-tree

    // =================================================================================================================
    // Sampling loop
    // =================================================================================================================
    int number_of_skipped_nodes = 0;
    while((tree.nodes_.size() + number_of_skipped_nodes) < maximum_rrt_samples) {

        auto [random_row, random_col] = (USE_INFORMED_RRT_STAR && path_to_goal_found) ?
                                        sample_from_heuristic_sampling_domain():
                                        sample_from_entire_grid();

        if(occupancy_grid.isGridCellOccupied(random_row, random_col)) {
            // grid cell is occupied
            continue;
        }

        // Run a knn-search (k=1 for RRT, k>1 for RRT*)
        const size_t k = USE_RRT_STAR ? rrt_star_nn_k : 1;
        size_t nn_indices[k];
        T nn_distances[k];
        KNNResultSet nn_results(k);
        nn_results.init(nn_indices, nn_distances);
        T query_position[2] = { random_row, random_col };
        const bool neighbors_found = index.findNeighbors(nn_results, query_position, nanoflann::SearchParams(10));

        size_t nn_index = nn_indices[0]; // for RRT (k=1)
        T nn_distance = nn_distances[0]; // for RRT (k=1)

        if(nn_distance == 0) {
            // skip cells that were already visited, but count them towards the total
            number_of_skipped_nodes++;
            continue;
        }

        // RRT* only: inspect the k-neighborhood, find minimum-cost link
        if(neighbors_found && USE_RRT_STAR) {

            std::vector<Node*> k_neighborhood;
            for(unsigned int i=0; i<k; i++) {
                const size_t j = nn_indices[i];
                assert(j < tree.nodes_.size());
                k_neighborhood.push_back(&tree.nodes_[j]);
            }
            const unsigned int argmin_cost = std::distance(k_neighborhood.begin(), std::min_element(
                    k_neighborhood.begin(), k_neighborhood.end(), [](Node* node0, Node* node1) {
                        return (node0->path_length_ < node1->path_length_); }));

            nn_index = nn_indices[argmin_cost];
            nn_distance = nn_distances[argmin_cost];
        }

        // The nearest node is our new parent
        const size_t parent_node_index = nn_index;
        const Node& parent_node = tree.nodes_[parent_node_index];
        const T parent_row = static_cast<T>(parent_node.position_(0));
        const T parent_col = static_cast<T>(parent_node.position_(1));

        assert(not occupancy_grid.isGridCellOccupied(parent_row, parent_col));

        // Expand the path from the parent to the leaf, check if any obstacles are in the way...
        cv::Vec2i leaf_position(0,0);
        const bool expansion_has_no_obstacles =
                occupancy_grid.expandPath(cv::Vec2i(parent_row, parent_col),
                           cv::Vec2i(random_row, random_col),
                           leaf_position,
                           maximum_rrt_expansion_distance);

        if(!expansion_has_no_obstacles) {
            continue;
        }

        const T leaf_row = leaf_position(0);
        const T leaf_col = leaf_position(1);

        assert(not occupancy_grid.isGridCellOccupied(leaf_row, leaf_col));

        const auto parent_to_leaf_distance = L1_norm(parent_row, parent_col, leaf_row, leaf_col);
        const auto accumulated_path_length_to_leaf = parent_node.path_length_ + parent_to_leaf_distance;

        tree.nodes_.push_back(Node(leaf_row, leaf_col, parent_node_index, accumulated_path_length_to_leaf)); // insert new leaf
        const Node& leaf_node = tree.nodes_.back();
        const size_t leaf_node_index = tree.nodes_.size() - 1;

        if(neighbors_found && USE_RRT_STAR) {

            for(unsigned int i=0; i<k; i++) {
                const size_t j = nn_indices[i];
                if(j==nn_index) {
                    continue; // we are only interested in the (k-1)-neighborhood
                }
                assert(j < tree.nodes_.size());
                Node& node = tree.nodes_[j];

                const auto node_to_leaf_distance = L1_norm(
                        node.position_(0), node.position_(1), leaf_row, leaf_col);

                if(accumulated_path_length_to_leaf + node_to_leaf_distance < node.path_length_) {
                    // re-link node to new parent (which is our leaf)

                    // Check if obstacles are in the way...
                    cv::Vec2i dummy_position;
                    const bool expansion_has_no_obstacles =
                            occupancy_grid.expandPath(cv::Vec2i(leaf_row, leaf_col),
                                       cv::Vec2i(node.position_(0), node.position_(1)),
                                       dummy_position,
                                       std::numeric_limits<int>::max());

                    if(!expansion_has_no_obstacles) {
                        continue;
                    }

                    node.path_length_ = accumulated_path_length_to_leaf + node_to_leaf_distance;
                    node.parent_ = leaf_node_index;

#ifndef NDEBUG
                    number_of_relinked_nodes++; // for debugging only
#endif
                }
            }
        }
        index.addPoints(tree.nodes_.size() - 1, tree.nodes_.size() - 1); // update kd-tree

        // Check if we are within reach of the goal
        const auto leaf_to_goal_distance = L1_norm(
                goal_row, goal_col, leaf_node.position_(0), leaf_node.position_(1));

        if(leaf_to_goal_distance <= rrt_goal_proximity_threshold) {
            path_to_goal_found = true;
            const int length_of_new_path_to_goal = leaf_node.path_length_ + leaf_to_goal_distance;
            length_of_best_path_to_goal = std::min<int>(length_of_best_path_to_goal, length_of_new_path_to_goal);

            if(USE_INFORMED_RRT_STAR) {
                // Uniform distribution across the best-fit bounding rectangle (!) of the elliptical heuristic
                // sampling domain described in [Gammell et al., 2014].

                // In the paper: sqrt(c_{best}^2 - c_{min}^2), where ...
                // c_{min}  ... linear shortest distance to goal (theoretical minimum cost)
                // c_{best} ... accumulated length of best path to goal (current best cost)

                // NOTE: If we are using the L1 distance metric, this is obviously just an
                //       approximation. Since the resulting L1 major_axis_length is an
                //       upper bound for the L2 minor_axis_length, this is ok.

                const float accumulated_length_of_best_path_to_goal = static_cast<float>(length_of_best_path_to_goal);

                // Use the variable naming scheme from [Gammell et al., 2014]...
                const float& c_min = shortest_linear_path_to_goal; // the L2 norm is the lower bound for the L1 norm
                const float& c_best = accumulated_length_of_best_path_to_goal;
                heuristic_sampling_domain_major_axis_length = static_cast<int>(c_best);
                //heuristic_sampling_domain_minor_axis_length = static_cast<int>(std::sqrt((c_best * c_best) - (c_min * c_min)));
                heuristic_sampling_domain_minor_axis_length = static_cast<int>(0.7071f * std::sqrt((c_best * c_best) - (c_min * c_min)));
                distribution_along_major_axis = UniformDistribution(static_cast<T>(0), static_cast<T>(heuristic_sampling_domain_major_axis_length));
                distribution_along_minor_axis = UniformDistribution(static_cast<T>(0), static_cast<T>(heuristic_sampling_domain_minor_axis_length));

                T_sampling_domain_to_grid_frame = Eigen::Affine3f::Identity();
                T_sampling_domain_to_grid_frame.pretranslate(Eigen::Vector3f(0, -static_cast<float>(heuristic_sampling_domain_minor_axis_length) / 2.0, 0.0f));
                T_sampling_domain_to_grid_frame.prerotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
                T_sampling_domain_to_grid_frame.pretranslate(Eigen::Vector3f(root_row, root_col, 0.0f));
            }
        }
    }

    // NOTE: If we did not find a path to the goal (i.e. path_to_goal_found==false), there are two possibilities:
    //       Return with an error, or choose the closest leaf in our tree and construct the path backwards from that.
    //       Let's go with the second option (which is what we would do anyways if path_to_goal_found==true).

    // Trace the path back from the goal to the parent node

    Path path; // path in the "map" coordinate frame
    GridPath grid_path; // path in the grid coordinate frame
    std::deque<Node> nodes_on_path;   // nodes on path (in the grid coordinate frame)

    // Run a knn-search (k=1) to find the closest node to our goal.
    const size_t k = 1;
    size_t nn_index;
    T nn_distance;
    KNNResultSet nn_results(k);
    nn_results.init(&nn_index, &nn_distance);
    T query_position[2] = { goal_row, goal_col };
    index.findNeighbors(nn_results, query_position, nanoflann::SearchParams(10));
    assert(nn_index < tree.nodes_.size());

    const auto closest_leaf_to_goal_distance = L1_norm(
            goal_row, goal_col, tree.nodes_[nn_index].position_(0), tree.nodes_[nn_index].position_(1));

    const auto accumulated_path_length_to_goal =
            tree.nodes_[nn_index].path_length_ + closest_leaf_to_goal_distance;

    nodes_on_path.push_front(Node(goal_row, goal_col, nn_index, accumulated_path_length_to_goal)); // insert the goal as the last node in the path
    grid_path.push_front(Eigen::Vector2i(goal_row, goal_col));

    while(true) {

        const size_t parent_node_index = nodes_on_path.front().parent_;
        const Node& parent_node = tree.nodes_[parent_node_index];
        nodes_on_path.push_front(parent_node);
        grid_path.push_front(parent_node.position_);

        const T node_row = nodes_on_path.front().position_(0);
        const T node_col = nodes_on_path.front().position_(1);

        const auto node_in_map_frame =
                T_grid_to_map * Eigen::Vector3f(node_row, node_col, 0.0f);

        path.push_front(Eigen::Vector2f(node_in_map_frame(0), node_in_map_frame(1)));
        if(parent_node_index==0) {
            break;
        }
    }

    bool should_generate_marker_messages = (marker_publisher.get() != nullptr);
    if(should_generate_marker_messages &&
       marker_publishing_task_.valid() &&
            marker_publishing_task_.wait_for(
               std::chrono::nanoseconds(0)) != std::future_status::ready) {
        should_generate_marker_messages = false;
    }

    if(should_generate_marker_messages) {
        assert((marker_publisher.get() != nullptr));

        marker_publishing_task_ = std::async(
                std::launch::async,
                [=]() {
                    std::vector<visualization_msgs::Marker> marker_messages = generateMarkerMessages(
                            tree,
                            path,
                            goal_in_map_frame,
                            T_grid_to_map,
                            Eigen::Vector2i(occupancy_grid.rows, occupancy_grid.cols),
                            T_sampling_domain_to_grid_frame,
                            Eigen::Vector2f(heuristic_sampling_domain_major_axis_length,
                                            heuristic_sampling_domain_minor_axis_length));
                    for(auto message : marker_messages) {
                        marker_publisher->publish(message);
                    }
                });
    }
    return {true, tree, path, grid_path};
}

std::vector<visualization_msgs::Marker>
        PathPlanner::generateMarkerMessages(
                const Tree tree,
                const Path path,
                const Eigen::Vector2f goal_in_map_frame,
                const Eigen::Affine3f& T_grid_to_map,
                const Eigen::Vector2i grid_size,
                const Eigen::Affine3f T_sampling_domain_to_grid,
                const Eigen::Vector2f sampling_domain_extents,
                const std::string map_frame
                ) const {

    int marker_message_id = 0;
    std::vector<visualization_msgs::Marker> marker_messages;
    
    const bool show_full_tree = true;
#define SHOW_FULL_TREE
#if defined(SHOW_FULL_TREE)
    if(show_full_tree) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame;
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker. This serves to create a unique ID.
        // Any marker sent with the same namespace and id will overwrite the old one.
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0 / 100.0f;
        marker.scale.y = 0.0; // must be zero for LINE_LIST
        marker.scale.z = 0.0; // must be zero for LINE_LIST

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.33f;

        marker.lifetime = ros::Duration();

        for(unsigned int j=0; j<tree.nodes_.size(); j++) {

            const size_t i = tree.nodes_[j].parent_;
            if(i==0) {
                continue;
            }

            const auto &vertex0 = tree.nodes_[i].position_;
            const auto &vertex1 = tree.nodes_[j].position_;

            const auto vertex0_in_map_frame =
                    T_grid_to_map *
                    Eigen::Vector3f(vertex0(0), vertex0(1), 0.0f);

            const auto vertex1_in_map_frame =
                    T_grid_to_map *
                    Eigen::Vector3f(vertex1(0), vertex1(1), 0.0f);

            geometry_msgs::Point p0;
            p0.x = vertex0_in_map_frame(0);
            p0.y = vertex0_in_map_frame(1);
            p0.z = 0.0;

            geometry_msgs::Point p1;
            p1.x = vertex1_in_map_frame(0);
            p1.y = vertex1_in_map_frame(1);
            p1.z = 0.0;

            marker.points.push_back(p0);
            marker.points.push_back(p1);
        }

        marker_messages.push_back(marker);
    }
#endif

    const bool show_heuristic_sampling_domain_bounds = true;

#define SHOW_HEURISTIC_SAMPLING_DOMAIN_BOUNDS
#if defined(SHOW_HEURISTIC_SAMPLING_DOMAIN_BOUNDS)
    if(show_heuristic_sampling_domain_bounds) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0 / 100.0f;
        marker.scale.y = 0.0; // must be zero for LINE_LIST
        marker.scale.z = 0.0; // must be zero for LINE_LIST

        marker.color.r = 255.0f / 255.0f;
        marker.color.g = 0.0f;
        marker.color.b = 200.0f / 255.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        std::vector<Eigen::Vector3f> vertices;

        const float major_axis_length = sampling_domain_extents(0);
        const float minor_axis_length = sampling_domain_extents(1);

        // vertices are initially in the "hyperellipsoid aligned frame".
        vertices.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
        vertices.push_back(Eigen::Vector3f(major_axis_length, 0.0f, 0.0f));
        vertices.push_back(Eigen::Vector3f(major_axis_length, minor_axis_length, 0.0f));
        vertices.push_back(Eigen::Vector3f(0.0f, minor_axis_length, 0.0f));

        std::vector<geometry_msgs::Point> points;

        for(auto &vertex : vertices) {

            // transform from "hyperellipsoid aligned frame" to "grid frame".
            const auto p_in_grid_frame = T_sampling_domain_to_grid * vertex;

            // transform from "grid frame" to "map frame".
            const auto p_in_map_frame = T_grid_to_map * p_in_grid_frame;
            geometry_msgs::Point p;
            p.x = p_in_map_frame(0);
            p.y = p_in_map_frame(1);
            p.z = 0.0f;
            points.push_back(p);
        }

        for(int i=0; i<points.size(); i++) {
            marker.points.push_back(points[i]);
            marker.points.push_back(points[(i+1) % points.size()]);
        }
        marker_messages.push_back(marker);
    }
#endif

    const bool show_occupancy_grid_outline = true;
#define SHOW_OCCUPANCY_GRID_OUTLINE
#if defined(SHOW_OCCUPANCY_GRID_OUTLINE)
    if(show_occupancy_grid_outline) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 1.0 / 100.0f;
        marker.scale.y = 0.0; // must be zero for LINE_LIST
        marker.scale.z = 0.0; // must be zero for LINE_LIST

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // vertices are initially in the "grid frame".
        std::vector<Eigen::Vector3f> vertices;

        vertices.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.0f));
        vertices.push_back(Eigen::Vector3f(grid_size(0)-1, 0.0f, 0.0f));
        vertices.push_back(Eigen::Vector3f(grid_size(0)-1, grid_size(1)-1, 0.0f));
        vertices.push_back(Eigen::Vector3f(0.0f, grid_size(1)-1, 0.0f));

        std::vector<geometry_msgs::Point> points;

        for(auto &vertex : vertices) {

            // transform from "grid frame" to "map frame".
            const auto p_in_map_frame = T_grid_to_map * vertex;
            geometry_msgs::Point p;
            p.x = p_in_map_frame(0);
            p.y = p_in_map_frame(1);
            p.z = 0.0f;
            points.push_back(p);
        }

        for(int i=0; i<points.size(); i++) {
            marker.points.push_back(points[i]);
            marker.points.push_back(points[(i+1) % points.size()]);
        }
        marker_messages.push_back(marker);
    }
#endif

    const bool show_path_to_goal = true;
#define SHOW_PATH_TO_GOAL
#if defined(SHOW_PATH_TO_GOAL)
    if(show_path_to_goal) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 4.0 / 100.0f;
        marker.scale.y = 0.0; // must be zero for LINE_LIST
        marker.scale.z = 0.0; // must be zero for LINE_LIST

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        assert(path.size() > 1); // must include at least the root and the goal
        for (unsigned int j = 1; j < path.size(); j++) {

            const auto &v0_in_map_frame = path[j-1];
            const auto &v1_in_map_frame = path[j];

            geometry_msgs::Point p0;
            p0.x = v0_in_map_frame(0); p0.y = v0_in_map_frame(1); p0.z = 0.0;
            marker.points.push_back(p0);

            geometry_msgs::Point p1;
            p1.x = v1_in_map_frame(0); p1.y = v1_in_map_frame(1); p1.z = 0.0;
            marker.points.push_back(p1);
        }
        marker_messages.push_back(marker);
    }
#endif

    const bool show_goal = true;
#define SHOW_GOAL
#if defined(SHOW_GOAL)
    if(show_goal) {

        visualization_msgs::Marker marker;
        marker.header.frame_id = map_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "rrt";
        marker.id = marker_message_id++;

        marker.type = visualization_msgs::Marker::SPHERE_LIST;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;

        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        geometry_msgs::Point p;
        p.x = goal_in_map_frame(0);
        p.y = goal_in_map_frame(1);
        p.z = 0.0;

        marker.points.push_back(p);
        marker_messages.push_back(marker);
    }
#endif
    return(marker_messages);
}

PathPlanner::Path GridToMap(
        const PathPlanner::GridPath& grid_path,
        const Eigen::Affine3f T_grid_to_map) {
    PathPlanner::Path path;
    for(int i=0; i<grid_path.size(); i++) {
        path.push_back((T_grid_to_map * Eigen::Vector3f(grid_path[i](0), grid_path[i](1), 0.0f)).head(2));
    }
    return(path);
}

PathPlanner::GridPath MapToGrid(
        const PathPlanner::Path& path,
        const Eigen::Affine3f T_map_to_grid) {
    PathPlanner::GridPath grid_path;
    for(int i=0; i<path.size(); i++) {
        grid_path.push_back((T_map_to_grid * Eigen::Vector3f(path[i](0), path[i](1), 0.0f)).cast<int>().head(2));
    }
    return(grid_path);
}
