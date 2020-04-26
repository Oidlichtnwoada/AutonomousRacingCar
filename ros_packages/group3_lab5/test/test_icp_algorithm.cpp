#include <gtest/gtest.h>
#include <scan_matcher/correspond.h>
#include <scan_matcher/transform.h>
#include <random>
#include <algorithm>

class TestICP : public ::testing::Test {
protected:
    typedef ::Point Point;
    typedef std::vector<::Point> Points;

    Points generateFakeLaserScan(bool random_phase = true, float angular_increment = 2.0f) {
        std::random_device random_device;
        std::mt19937 random_generator(random_device());
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

    Transform matchPointSets(const Points& pts_t0, const Points& pts_t1) {
        Points temp_pts_t0 = pts_t0;
        Points temp_pts_t1 = pts_t1;
        std::vector<std::vector<int> > jump_table;
        computeJump(jump_table, temp_pts_t0);
        vector<Correspondence> correspondence_vector;
        getCorrespondence(temp_pts_t0, temp_pts_t1, temp_pts_t1, jump_table, correspondence_vector, 0);
        Transform transform;
        updateTransform(correspondence_vector, transform);
        return(transform);
    }
};

TEST_F(TestICP, check_transformation_estimation) {

    const Transform known_transform(1.0, 0.0, 0.0);
    Points pts_t0 = generateFakeLaserScan();
    Points pts_t1 = transformPointSet(pts_t0, 1.0, 0.0, 0.0);
    const Transform estimated_transform = matchPointSets(pts_t0, pts_t1);
    EXPECT_TRUE(estimated_transform == known_transform);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
