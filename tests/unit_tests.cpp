#include "gtest/gtest.h"
#include "lingeo.hpp"

bool equal_vectors(const std::vector<lingeo::point_t>& vec1, const std::vector<lingeo::point_t>& vec2) {
    if (vec1.size() != vec2.size()) {
        return false;
    }
    for (size_t i = 0; i < vec1.size(); i++) {
        if (!vec1[i].equals(vec2[i])) {
            return false;
        }
    }
    return true;
}

TEST(intersections, test1) {
    std::vector<float> vec1 = { 0.f, 0.f, 1.f, 1.f, 10.f, 2.f, 11.f, -3.f };
    std::vector<float> vec2 = { 1.f, 1.f, 11.f, 1.f, 5.f, 5.f};

    lingeo::convex_polygon_t p1(vec1);
    lingeo::convex_polygon_t p2(vec2);

    lingeo::convex_polygon_t p3 = p1.intersection(p2);

    ASSERT_EQ(p3.size(), 4);
    ASSERT_TRUE(equal_vectors(p3.get_vertices(), { {10.2f, 1.f}, {10.0769f, 1.61538f}, {9.57143f, 1.95238f}, {1.f, 1.f} }));
    ASSERT_FLOAT_EQ(p3.square(), 4.5157495f);
}

TEST(intersections, test2) {
    ASSERT_EQ(1, 1);

    std::vector<float> vec1 = { 0.f, 0.f, 3.f, 3.f, 7.f, 3.f, 10.f, 0.f };
    std::vector<float> vec2 = { 7.f, 3.f, 10.f, 0.f, 11.f, 10.f };

    lingeo::convex_polygon_t p1(vec1);
    lingeo::convex_polygon_t p2(vec2);

    lingeo::convex_polygon_t p3 = p1.intersection(p2);

    ASSERT_FLOAT_EQ(p3.square(), 0.f);
}

TEST(intersections, test3) {
    ASSERT_EQ(1, 1);

    std::vector<float> vec1 = { 0.f, 0.f, 3.f, 3.f, 7.f, 3.f, 10.f, 0.f };
    std::vector<float> vec2 = { 10.f, 0.f, 11.f, 0.f, 11.f, -1.f, 10.f, -1.f };

    lingeo::convex_polygon_t p1(vec1);
    lingeo::convex_polygon_t p2(vec2);

    lingeo::convex_polygon_t p3 = p1.intersection(p2);

    ASSERT_FLOAT_EQ(p3.square(), 0.f);
}

TEST(intersections, test4) {
    ASSERT_EQ(1, 1);

    std::vector<float> vec1 = { 0.f, 0.f, 3.f, 3.f, 7.f, 3.f, 10.f, 0.f };
    std::vector<float> vec2 = { 0.f, 20.f, 3.f, 3.f, 3.f, 0.f, 0.f, 0.f };

    lingeo::convex_polygon_t p1(vec1);
    lingeo::convex_polygon_t p2(vec2);

    lingeo::convex_polygon_t p3 = p1.intersection(p2);

    ASSERT_FLOAT_EQ(p3.square(), 4.5f);
}

TEST(intersections, test5) {
    ASSERT_EQ(1, 1);

    std::vector<float> vec1 = { 0.f, 0.f, 3.f, 3.f, 7.f, 3.f, 10.f, 0.f };
    std::vector<float> vec2 = { 0.f, 20.f, 3.f, 3.f, 2.f, 0.f, 0.f, 0.f };

    lingeo::convex_polygon_t p1(vec1);
    lingeo::convex_polygon_t p2(vec2);

    lingeo::convex_polygon_t p3 = p1.intersection(p2);
    
    ASSERT_FLOAT_EQ(p3.square(), 3.f);
}
