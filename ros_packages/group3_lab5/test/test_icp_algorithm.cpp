#include <gtest/gtest.h>
#include <scan_matcher/correspond.h>
#include <scan_matcher/transform.h>
#include <random>
#include <algorithm>

#define TRANSFORM_EPSILON 1E-4
#define MAX_NUMBER_OF_ITERATIONS 100
#define NUMBER_OF_TESTS 100

class TestICP : public ::testing::Test {
protected:
    typedef ::Point Point;
    typedef std::vector<::Point> Points;

    Points generateFakeLaserScan(bool random_phase = true, float angular_increment = 2.0f) {
        std::random_device random_device;
        std::mt19937 random_generator(random_device());
        random_generator.seed(123456789UL); // constant seed for reproducible results
        std::uniform_real_distribution<float> random_distribution(0.0f, M_PI);
        const std::vector<float> frequencies = {2,3,5,13};
        std::vector<float> phases(frequencies.size(), 0.0f);
        if(random_phase) {
            std::generate(phases.begin(), phases.end(), [&]() mutable {
                return random_distribution(random_generator);
            });
        }

        assert(std::fmod(360.0, angular_increment) == 0.0f);

        Points pts;
        for(float angle_deg = 0.0;
            angle_deg <= 360.0f;
            angle_deg += angular_increment) {

            const float theta = angle_deg / 180.0f * M_PI;
            float rho = 0.0f;

            for(unsigned int i=0; i<frequencies.size(); i++) {
                rho += std::abs(std::cos(phases[i] + theta * frequencies[i]));
            }
            pts.push_back(Point(rho, theta));
        }

        const float x0 = pts.front().getX();
        const float y0 = pts.front().getY();
        const float x1 = pts.back().getX();
        const float y1 = pts.back().getY();
        assert(std::abs(x0-x1) < 1E-3 && std::abs(y0-y1) < 1E-3);
        pts.pop_back();
        return(pts);
    }

    Points transformPointSet(const Points& pts, float x, float y, float theta) {
        Points transformed_pts;
        for(unsigned int i=0; i<pts.size(); i++) {
            Point p = pts[i];
            p.rotate(theta);
            p.translate(x,y);
            transformed_pts.push_back(p);
        }
        return(transformed_pts);
    }
};

TEST_F(TestICP, check_transformation_estimation) {

    const float tx = 0.1f;
    const float ty = -0.05f;
    const float rot_theta = 4.0 / 180.0 * M_PI; // 4 deg.

    Points pts_t0 = generateFakeLaserScan(false, 1.0f);
    const Transform known_transform(tx,ty,rot_theta);
    Points pts_t1 = transformPointSet(pts_t0,tx,ty,rot_theta);

    SimpleCorrespondences correspondences;
    for(int i=0; i<pts_t0.size(); i++) {

        const int j2_up = i + 1;
        const int j2_down = i - 1;
        int j2 = (-1);

        if (j2_up < pts_t0.size()) {
            j2 = j2_up;
        }
        if (j2_down >= 0) {

            if (j2 != (-1)) {
                const Point &p_j2_up = pts_t0[j2_up];
                const Point &p_j2_down = pts_t0[j2_down];
                const float distance_j2_up = pts_t1[i].distToPoint2(&p_j2_up);
                const float distance_j2_down = pts_t1[i].distToPoint2(&p_j2_down);
                j2 = (distance_j2_up < distance_j2_down) ? j2_up : j2_down;
            } else {
                j2 = j2_down;
            }
        }

        const Point& p_t0 = pts_t0[i];
        const Point& p_t0_second_best = pts_t0[j2];
        const Point& p_t1 = pts_t1[i];

        const SimpleCorrespondence correspondence(
                Eigen::Vector2f(p_t0.getX(), p_t0.getY()),
                Eigen::Vector2f(p_t0_second_best.getX(), p_t0_second_best.getY()),
                Eigen::Vector2f(p_t1.getX(), p_t1.getY()));
        correspondences.push_back(correspondence);
    }
    Transform estimated_transform = estimateTransformation(correspondences);
    EXPECT_TRUE(estimated_transform == known_transform);
}

TEST_F(TestICP, check_full_algorithm) {

    std::random_device random_device;
    std::mt19937 random_generator(random_device());
    random_generator.seed(987654321UL); // constant seed for reproducible results
    std::uniform_real_distribution<float> random_distribution(0.0f, 1.0);

    for(int test_n = 0; test_n < NUMBER_OF_TESTS; test_n++) {

        const float tx = random_distribution(random_generator) - 0.5;
        const float ty = random_distribution(random_generator) - 0.5;
        const float rot_theta = random_distribution(random_generator) / 180.0f * M_PI * 15.0;

        const Transform known_transform(tx, ty, rot_theta);
        Points pts_t0 = generateFakeLaserScan(true, 1.0f);
        Points pts_t1 = transformPointSet(pts_t0, tx, ty, rot_theta);

        Transform estimated_transform_t1_to_t0(0.f, 0.f, 0.f);
        const JumpTable jump_table = computeJumpTable(pts_t0);

        unsigned int number_of_iterations = 0;

        for (int i = 0; i < MAX_NUMBER_OF_ITERATIONS; i++) {

            number_of_iterations++;
            Points trans_pts_t1 = transformPoints(pts_t1, estimated_transform_t1_to_t0);
            SimpleCorrespondences correspondences = findCorrespondences(pts_t0, trans_pts_t1, jump_table);
            const Transform new_estimated_transform_t1_to_t0 =
                    estimated_transform_t1_to_t0 +
                    estimateTransformation(correspondences, false);

            if (estimated_transform_t1_to_t0 == new_estimated_transform_t1_to_t0) {
                break;
            }
            estimated_transform_t1_to_t0 = new_estimated_transform_t1_to_t0;
        }

        const float inverse_tx = estimated_transform_t1_to_t0.x_disp;
        const float inverse_ty = estimated_transform_t1_to_t0.y_disp;
        const float inverse_rot_theta = estimated_transform_t1_to_t0.theta_rot;

        const bool tx_is_correct = std::abs(tx + inverse_tx) < TRANSFORM_EPSILON;
        const bool ty_is_correct = std::abs(ty + inverse_ty) < TRANSFORM_EPSILON;
        const bool rot_theta_is_correct = std::abs(rot_theta + inverse_rot_theta) < TRANSFORM_EPSILON;

        if(!tx_is_correct || !ty_is_correct || !rot_theta_is_correct) {
            assert(false);
        }

        EXPECT_TRUE(tx_is_correct);
        EXPECT_TRUE(ty_is_correct);
        EXPECT_TRUE(rot_theta_is_correct);
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
