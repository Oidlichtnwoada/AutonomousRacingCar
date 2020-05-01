#include <scan_matcher/correspond.h>

// Enable this for "naive" correspondence search (without jump tables):
// #define DISABLE_OPTIMIZATIONS

Point::Point() : r(0), theta(0) {}

Point::Point(float range, float angle) : r(range), theta(angle){};

float Point::distToPoint(const Point* pt2) const {
    return sqrt(r*r + pt2->r*pt2->r - 2*r*pt2->r*cos(pt2->theta-theta));
}

float Point::distToPoint2(const Point* pt2) const {
    return r*r + (pt2->r)*(pt2->r) - 2*r*(pt2->r)*std::cos(pt2->theta - theta);
}

float Point::radialGap(const Point* pt2) const{
    return abs(r-pt2->r);
}

float Point::getX() const { return r * cos(theta); }
float Point::getY() const { return r * sin(theta); }

bool Point::operator<(const Point& p) const { return theta < p.theta; }
bool Point::operator>(const Point& p) const { return theta > p.theta; }

void Point::wrapTheta(){
    while(theta > M_PI){
        theta -= 2*M_PI;
    }
    while(theta <= -M_PI){
        theta += 2*M_PI;
    }
}

void Point::rotate(float phi){
    theta = theta + phi;
    wrapTheta();
}

void Point::translate(float x, float y) {
    float newx = getX()+x;
    float newy = getY()+y;
    r = sqrt(newx*newx+newy*newy);
    theta = atan2(newy,newx);
}

Eigen::Vector2f Point::getVector() const {
    return Eigen::Vector2f(getX(), getY());
}

geometry_msgs::Point Point::getPoint() const {
    geometry_msgs::Point p;
    p.x = r * cos(theta); p.y = r * sin(theta); p.z = 0.0;
    return p;
}

SimpleCorrespondence::SimpleCorrespondence(Eigen::Vector2f p0,
                                           Eigen::Vector2f p0_second_best,
                                           Eigen::Vector2f p1,
                                           unsigned int idx_p0,
                                           unsigned int idx_p1,
                                           float distance)
    : p_t0_(p0), p_t1_(p1), idx_p0_(idx_p0), idx_p1_(idx_p1), distance_(distance) {
    nn_ << p0.y() - p0_second_best.y(), p0_second_best.x() - p0.x();
    nn_.normalize();
}

const Eigen::Vector2f& SimpleCorrespondence::p_t0() const { return p_t0_; }
const Eigen::Vector2f& SimpleCorrespondence::p_t1() const { return p_t1_; }
const Eigen::Vector2f& SimpleCorrespondence::nn() const   { return nn_; }
unsigned int SimpleCorrespondence::idx_p0() const   { return idx_p0_; }
unsigned int SimpleCorrespondence::idx_p1() const   { return idx_p1_; }
float SimpleCorrespondence::distance() const        { return distance_; }


JumpTable computeJumpTable(const Points& points) {
    const int number_of_points = points.size();
    JumpTable jt;

    for(int i=0; i < number_of_points; i++) {

        JumpTableEntry jte;

        int j = i + 1;
        while(j < number_of_points && points[j].r <= points[i].r) j++;
        jte.up_bigger = (j-i);

        j = i + 1;
        while(j < number_of_points && points[j].r >= points[i].r) j++;
        jte.up_smaller = (j-i);

        j = i - 1;
        while(j>=0 && points[j].r>=points[i].r) j--;
        jte.down_smaller = (j-i);

        j = i - 1;
        while(j>=0 && points[j].r<=points[i].r) j--;
        jte.down_bigger = (j-i);

        jt.push_back(jte);
    }
    return(jt);
}

SimpleCorrespondences findCorrespondences(const Points& pts_t0,
                                          const Points& trans_pts_t1,  // pts_t1 transformed to pts_t0's frame
                                          const JumpTable& jump_table,  // computed from pts_t0
                                          const float max_correspondence_dist) {


    SimpleCorrespondences correspondences;

#define INVALID_INDEX -1
    assert(jump_table.size() == pts_t0.size());

    const int number_of_rays_t0 = pts_t0.size();
    const int number_of_rays_t1 = trans_pts_t1.size();
    int index_of_last_best_match = INVALID_INDEX; // this is an index into pts_t0

    std::vector<float> minimum_distance_so_far(number_of_rays_t0, std::numeric_limits<float>::max()); // used for outlier removal

    for (unsigned int i = 0; i < number_of_rays_t1; i++) { // i is an index into trans_pts_t1

        const Point p_i_w = trans_pts_t1[i]; // i-th point of y_{t} in y_{t-1}'s frame
        // in [Censi 2008]: p_{i}^{$\omega$}

        int index_of_best_match = INVALID_INDEX; // this is an index into pts_t0

        float distance_of_best_match = std::numeric_limits<float>::max();

        // approximated index in pts_t0 corresponding to trans_pts_t1[i]
        int start_index = i; // [TP]: for our purposes, this can be set to i

        const int we_start_at = (index_of_last_best_match != INVALID_INDEX) ? (index_of_last_best_match + 1)
                                                                            : (start_index);

        assert(we_start_at >= 0);
        int j_up = std::min<int>(we_start_at + 1, number_of_rays_t0-1); // j_up is an index into pts_t0
        int j_down = std::min<int>(we_start_at, number_of_rays_t0-1);   // j_up is an index into pts_t0

        // distance of the last point examined in up/down direction
        float last_distance_in_up_direction = std::numeric_limits<float>::max();
        float last_distance_in_down_direction = std::numeric_limits<float>::max();

        bool up_search_stopped = false;
        bool down_search_stopped = false;

        while (!up_search_stopped || !down_search_stopped) {

            const bool should_search_in_up_direction =
                    !up_search_stopped ||
                    (down_search_stopped && (last_distance_in_up_direction < last_distance_in_down_direction));
            // NOTE: This condition not correctly stated in Fig.5 of [Censi 2008], resulting in bugs
            //       when copied verbatim from the paper.

            if (should_search_in_up_direction) {
                if (j_up >= number_of_rays_t0) {
                    up_search_stopped = true;
                    continue;
                } else {

                    const Point &p_j = pts_t0[j_up];
                    last_distance_in_up_direction = p_i_w.distToPoint2(&p_j);

                    const bool correspondence_is_acceptable = true;
                    // this is from [Censi 2008], but it's not clear
                    // what the acceptance criteria should be...

                    if (correspondence_is_acceptable &&
                        (last_distance_in_up_direction < distance_of_best_match)) {
                        index_of_best_match = j_up;
                        distance_of_best_match = last_distance_in_up_direction;
                    }

#ifndef DISABLE_OPTIMIZATIONS
                    if (j_up > start_index) {

                        // compute early stopping criteria

                        const float delta_theta = (p_j.theta - p_i_w.theta);
                        const float min_dist_up = p_i_w.r * ((delta_theta > M_PI_2) ? 1.0 : std::sin(delta_theta));

                        if((min_dist_up * min_dist_up) > distance_of_best_match) {
                            up_search_stopped = true;
                            continue;
                        }

                        // jump table lookup
                        if(p_j.r < p_i_w.r) {
                            j_up += jump_table[j_up].up_bigger;
                        } else {
                            j_up += jump_table[j_up].up_smaller;
                        }

                        if (j_up >= number_of_rays_t0) {
                            up_search_stopped = true;
                            continue;
                        }

                    } else {
                        j_up++;
                    }
#else
                    j_up++;
#endif
                }
            }

            const bool should_search_in_down_direction =
                    !down_search_stopped ||
                    (up_search_stopped && (last_distance_in_up_direction >= last_distance_in_down_direction));

            if (should_search_in_down_direction) {

                if (j_down < 0) {
                    down_search_stopped = true;
                    continue;
                } else {

                    const Point &p_j = pts_t0[j_down];
                    last_distance_in_down_direction = p_i_w.distToPoint2(&p_j);

                    const bool correspondence_is_acceptable = true;
                    // this is from [Censi 2008], but it's not clear
                    // what the acceptance criteria should be...

                    if (correspondence_is_acceptable &&
                        (last_distance_in_down_direction < distance_of_best_match)) {
                        index_of_best_match = j_down;
                        distance_of_best_match = last_distance_in_down_direction;
                    }

#ifndef DISABLE_OPTIMIZATIONS
                    if (j_down < start_index) {
                        // compute early stopping criteria

                        const float delta_theta = (p_i_w.theta - p_j.theta);
                        const float min_dist_down = p_i_w.r * ((delta_theta > M_PI_2) ? 1.0 : std::sin(delta_theta));

                        if((min_dist_down * min_dist_down) > distance_of_best_match) {
                            down_search_stopped = true;
                            continue;
                        }

                        // jump table lookup
                        if(p_j.r < p_i_w.r) {
                            j_down += jump_table[j_down].down_bigger;
                        } else {
                            j_down += jump_table[j_down].down_smaller;
                        }

                        if (j_down < 0) {
                            down_search_stopped = true;
                            continue;
                        }


                    } else {
                        j_down--;
                    }
#else
                    j_down--;
#endif
                }
            }

            index_of_last_best_match = index_of_best_match;
        }

        // Find j2, the index of the "second best match" (in the immediate neighborhood of the best match)

        const int j2_up = index_of_last_best_match + 1; // j2_up is an index into pts_t0
        const int j2_down = index_of_last_best_match - 1; // j2_down is an index into pts_t0
        int j2 = INVALID_INDEX;

        if (j2_up < number_of_rays_t0) {
            j2 = j2_up;
        }
        if (j2_down >= 0) {

            if (j2 != INVALID_INDEX) {
                const Point &p_j2_up = pts_t0[j2_up];
                const Point &p_j2_down = pts_t0[j2_down];
                const float distance_j2_up = p_i_w.distToPoint2(&p_j2_up);
                const float distance_j2_down = p_i_w.distToPoint2(&p_j2_down);
                j2 = (distance_j2_up < distance_j2_down) ? j2_up : j2_down;
            } else {
                j2 = j2_down;
            }
        }

        const int index_of_second_best_match = j2;

        if (index_of_last_best_match != INVALID_INDEX &&
            index_of_second_best_match != INVALID_INDEX &&
            distance_of_best_match <= max_correspondence_dist) {

            const Point& p_t0 = pts_t0[index_of_last_best_match];
            const Point& p_t0_second_best = pts_t0[index_of_second_best_match];
            const Point& p_t1 = trans_pts_t1[i];

            const SimpleCorrespondence correspondence(
                    Eigen::Vector2f(p_t0.getX(), p_t0.getY()),
                    Eigen::Vector2f(p_t0_second_best.getX(), p_t0_second_best.getY()),
                    Eigen::Vector2f(p_t1.getX(), p_t1.getY()),
                    index_of_last_best_match, i, distance_of_best_match);
            correspondences.push_back(correspondence);
            minimum_distance_so_far[index_of_last_best_match] = std::min(minimum_distance_so_far[index_of_last_best_match], distance_of_best_match); // used for outlier removal
        }
    } // main loop (i)

    // outlier removal
    SimpleCorrespondences correspondences_without_outliers;
    for(const SimpleCorrespondence& correspondence : correspondences) {
        if(minimum_distance_so_far[correspondence.idx_p0()] == correspondence.distance()) {
            correspondences_without_outliers.push_back(correspondence);
        }
    }
    return(correspondences_without_outliers);
}