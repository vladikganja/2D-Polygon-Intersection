#pragma once
#include "lingeo.hpp"

int main() {
    /*
    
    7
    6
    1 1 5 1 7 2 5 4 1 4 3 3 7 10
    2 2 5 2 7 4 6 6 2 6 4 6

    */

    int N1;
    int N2;
    std::cin >> N1 >> N2;

    std::vector<lingeo::point_t> first_vertices;
    std::vector<lingeo::point_t> second_vertices;

    for (int i = 0; i < N1; i++) {
        float x, y;
        std::cin >> x >> y;
        first_vertices.push_back(lingeo::point_t(x, y));
    }

    for (int i = 0; i < N2; i++) {
        float x, y;
        std::cin >> x >> y;
        second_vertices.push_back(lingeo::point_t(x, y));
    }

    lingeo::convex_polygon_t t1(first_vertices);
    lingeo::convex_polygon_t t2(second_vertices);

    lingeo::convex_polygon_t t3 = t1.intersection(t2);
    t3.print();

    std::cout << t3.square() << std::endl;

    t1.add_vertex(lingeo::point_t(7.0, 10.0));
}
