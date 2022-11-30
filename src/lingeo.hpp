#pragma once
#include <iostream>
#include <vector>
#include <algorithm>
#include <cassert>

namespace lingeo {

class point_t;
class line_t;
class convex_polygon_t;

namespace utils {
    float deviation_value = 0.000'1f;

    enum class Semispace {
        UPPER,
        LOWER,
        SAME_LINE
    };
};

class point_t {
public:
    float x_;
    float y_;

    point_t() {
        x_ = NAN;
        y_ = NAN;
    }

    point_t(const point_t& other) {
        x_ = other.x_;
        y_ = other.y_;
    }

    point_t(point_t&& other) noexcept {
        x_ = other.x_;
        y_ = other.y_;
        other.x_ = 0.f;
        other.y_ = 0.f;
    }

    point_t(float x, float y) {
        x_ = x;
        y_ = y;
    }

    point_t& operator=(const point_t& other) {
        x_ = other.x_;
        y_ = other.y_;
        return *this;
    }

    point_t& operator=(point_t&& other) noexcept {
        x_ = other.x_;
        y_ = other.y_;
        other.x_ = 0.f;
        other.y_ = 0.f;
        return *this;
    }

    bool is_valid() const {
        return x_ != NAN && y_ != NAN;
    }

    bool equals(const point_t& other) const {
        return (
            std::abs(x_ - other.x_) < utils::deviation_value &&
            std::abs(y_ - other.y_) < utils::deviation_value
        );
    }
};

namespace utils {
    float vector_multiplication(const point_t& p1, const point_t& p2, point_t base = point_t(0.f, 0.f)) {
        return (p1.x_ - base.x_) * (p2.y_ - base.y_) - (p2.x_ - base.x_) * (p1.y_ - base.y_);
    }
}

class line_t {
private:
    float a_ = NAN;
    float b_ = NAN;
    float c_ = NAN;

public:

    line_t() = delete;
    line_t(const line_t& other) = delete;
    line_t(line_t&& other) = delete;

    line_t(const point_t& a, const point_t& b) {
        assert(a.is_valid() && b.is_valid());
        ///////////////////////////////////////////////////////
        // 
        // Ax + By + C = 0
        // A = y2 - y1
        // B = x1 - x2
        // C = x2*y1 - x1*y2
        //
        ///////////////////////////////////////////////////////

        a_ = b.y_ - a.y_;
        b_ = a.x_ - b.x_;
        c_ = b.x_ * a.y_ - a.x_ * b.y_;
    }

    utils::Semispace calculate_point_semispace(const point_t& p) const {
        assert(p.is_valid());

        float val = a_ * p.x_ + b_ * p.y_ + c_;
        if (val < 0.f - utils::deviation_value) {
            return utils::Semispace::LOWER;
        }
        else if (val > 0.f + utils::deviation_value) {
            return utils::Semispace::UPPER;
        }
        return utils::Semispace::SAME_LINE;
    }

    point_t calculate_point_of_intersection(const line_t& other) const {
        point_t res;
        res.x_ = (b_ * other.c_ - other.b_ * c_) / (a_ * other.b_ - other.a_ * b_);
        res.y_ = (other.a_ * c_ - a_ * other.c_) / (a_ * other.b_ - other.a_ * b_);

        return res;
    }
};

class convex_polygon_t {
private:
    std::vector<point_t> vertices_;

    void add_convex_vertex(const point_t& p) {
        vertices_.push_back(p);
    }

    // guarantees object consistency
    void validate_convex_hull() {
        if (vertices_.size() < 3) {
            throw std::logic_error("ERROR::Polygon can't contain less than 3 points.\n");
        }

        point_t base(INFINITY, INFINITY);

        for (auto& vertex : vertices_) {
            if (vertex.x_ < base.x_) {
                base.x_ = vertex.x_;
                base.y_ = vertex.y_;
            }
        }

        // sorting points by angle
        std::sort(vertices_.begin(), vertices_.end(), [&](const point_t& p1, const point_t& p2) -> bool {
            if (p1.equals(base)) {
                return true;
            }
            if (p2.equals(base)) {
                return false;
            }
            float vect_mult_val = utils::vector_multiplication(p1, p2, base);
            if (std::abs(vect_mult_val) < utils::deviation_value) {
                return (
                    std::pow((p1.x_ - base.x_), 2.f) + std::pow((p1.y_ - base.y_), 2.f) <
                    std::pow((p2.x_ - base.x_), 2.f) + std::pow((p2.y_ - base.y_), 2.f)
                );
            }
            return vect_mult_val > 0.f;
        });

        // Graham scan
        std::vector<point_t> convex_hull{vertices_[0], vertices_[1]};
        for (size_t i = 2; i < vertices_.size(); i++) {
            float vect_mult_val = utils::vector_multiplication(
                vertices_[i], 
                convex_hull[convex_hull.size() - 1], 
                convex_hull[convex_hull.size() - 2]
            );

            if (vect_mult_val < 0.f) {
                convex_hull.push_back(vertices_[i]);
            }
            else {
                while (vect_mult_val >= 0.f) {
                    convex_hull.pop_back();
                    if (convex_hull.size() == 1) {
                        break;
                    }
                    vect_mult_val = utils::vector_multiplication(
                        vertices_[i], 
                        convex_hull[convex_hull.size() - 1], 
                        convex_hull[convex_hull.size() - 2]
                    );
                }
                convex_hull.push_back(vertices_[i]);
            }
        }

        vertices_ = std::move(convex_hull);

        if (vertices_.size() < 3) {
            throw std::logic_error("ERROR::Polygon can't contain less than 3 points.\n");
        }
    }

public:

    convex_polygon_t() {}

    convex_polygon_t(const std::vector<point_t>& vec, bool safe_mode = true) {
        vertices_ = vec;
        if (safe_mode) {
            validate_convex_hull();
        }
    }

    convex_polygon_t(const std::vector<float>& vec, bool safe_mode = true) {
        assert(vec.size() >= 2);
        for (size_t i = 0; i < vec.size() - 1; i += 2) {
            vertices_.push_back(point_t(vec[i], vec[i + 1]));
        }
        if (safe_mode) {
            validate_convex_hull();
        }
    }

    convex_polygon_t(const convex_polygon_t& other) {
        vertices_ = other.vertices_;
    }

    convex_polygon_t(convex_polygon_t&& other) noexcept {
        vertices_ = std::move(other.vertices_);
    }

    convex_polygon_t(std::vector<point_t>&& vec, bool safe_mode = true) noexcept {
        vertices_ = std::move(vec);
        if (safe_mode) {
            validate_convex_hull();
        }
    }

    convex_polygon_t& operator=(const convex_polygon_t& other) {
        vertices_ = other.vertices_;
        return *this;
    }

    convex_polygon_t& operator=(convex_polygon_t&& other) noexcept {
        vertices_ = std::move(other.vertices_);
        return *this;
    }

    ~convex_polygon_t() {}

    // Sutherland-Hodgman algorithm
    convex_polygon_t intersection(const convex_polygon_t& other) const {
        convex_polygon_t cutting_shape(*this);

        // the order of vertices doesn't matter
        for (size_t v = 0; v < other.vertices_.size(); v++) {
            line_t cutting_line(other.vertices_[v], other.vertices_[(v + 1) % other.vertices_.size()]);
            point_t opposite_point = other.vertices_[(v + 2) % other.vertices_.size()];
            utils::Semispace desired_semispace = cutting_line.calculate_point_semispace(opposite_point);

            convex_polygon_t tmp_res;
            auto& cutting = cutting_shape.vertices_;
            for (size_t ver = 0; ver < cutting.size(); ver++) {
                utils::Semispace first_semispace = cutting_line.calculate_point_semispace(cutting[ver]);
                utils::Semispace second_semispace = cutting_line.calculate_point_semispace(cutting[(ver + 1) % cutting.size()]);

                if (first_semispace == utils::Semispace::SAME_LINE) {
                    first_semispace = desired_semispace;
                }
                if (second_semispace == utils::Semispace::SAME_LINE) {
                    second_semispace = desired_semispace;
                }

                if (first_semispace == desired_semispace && second_semispace == desired_semispace) {
                    tmp_res.add_convex_vertex(cutting[ver]);
                }
                else if (first_semispace == desired_semispace && second_semispace != desired_semispace) {
                    tmp_res.add_convex_vertex(cutting[ver]);
                    line_t tmp(cutting[ver], cutting[(ver + 1) % cutting.size()]);
                    point_t intersection_point = cutting_line.calculate_point_of_intersection(tmp);
                    if (intersection_point.is_valid() && !intersection_point.equals(cutting[ver])) {
                        tmp_res.add_convex_vertex(intersection_point);
                    }
                }
                else if (first_semispace != desired_semispace && second_semispace == desired_semispace) {
                    line_t tmp(cutting[ver], cutting[(ver + 1) % cutting.size()]);
                    point_t intersection_point = cutting_line.calculate_point_of_intersection(tmp);
                    if (intersection_point.is_valid()) {
                        tmp_res.add_convex_vertex(intersection_point);
                    }
                }
                else {
                    /*nothing to do*/
                }
            }

            cutting_shape = std::move(tmp_res);
        }

        return cutting_shape;
    }

    // Gauss's Shoelace formula
    float square() const {
        float sq = 0.f;
        for (size_t ver = 0; ver < vertices_.size(); ver++) {
            sq += utils::vector_multiplication(vertices_[ver], vertices_[(ver + 1) % vertices_.size()]);
        }

        return std::abs(sq) / 2.f;
    }

    void print() const {
        for (auto& ver : vertices_) {
            std::cout << ver.x_ << "; " << ver.y_ << std::endl;
        }
        std::cout << std::endl;
    }

    void add_vertex(const point_t& p, bool safe_mod = true) {
        assert(p.is_valid());
        vertices_.push_back(p);
        if (safe_mod) {
            validate_convex_hull();
        }
    }

    size_t size() const {
        return vertices_.size();
    }

    std::vector<point_t> get_vertices() const {
        return vertices_;
    }
};

}
