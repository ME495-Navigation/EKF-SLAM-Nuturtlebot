#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <string>
#include <sstream>


using namespace turtlelib;
const double tol = 0.0001;

TEST_CASE( "Normalize Angle", "[normalize_angle]" )
{
    REQUIRE_THAT( normalize_angle(PI), Catch::Matchers::WithinAbs(PI, tol) );
    REQUIRE_THAT( normalize_angle(-PI), Catch::Matchers::WithinAbs(PI, tol) );
    REQUIRE_THAT( normalize_angle(0.0), Catch::Matchers::WithinAbs(0.0, tol) );
    REQUIRE_THAT( normalize_angle(-PI/4), Catch::Matchers::WithinAbs(-PI/4, tol) );
    REQUIRE_THAT( normalize_angle((3*PI)/2), Catch::Matchers::WithinAbs(-PI/2, tol) );
    REQUIRE_THAT( normalize_angle(-(5*PI)/2), Catch::Matchers::WithinAbs(-PI/2, tol) );

}

TEST_CASE( "Output Operator", "[operator<<]" )
{
    Point2D p = {0.4, 0.6};
    std::ostringstream sp;
    sp << p;
    REQUIRE(sp.str() == "[0.4, 0.6]");

    Vector2D v;
    std::ostringstream sv;
    sv << v;
    REQUIRE(sv.str() == "[0, 0]");
}

TEST_CASE( "Input Operator", "[operator>>]" )
{
    Point2D p2;
    std::istringstream sp2("[0.4, 0.6]");
    sp2 >> p2;
    REQUIRE(sp2.str() == "[0.4, 0.6]");

    Vector2D v2;
    std::istringstream sv2("[0, 0]");
    sv2 >> v2;
    REQUIRE(sv2.str() == "[0, 0]");
}

TEST_CASE( "Add Operator", "[operator+]" )
{
    Point2D tail = {1.0, 2.0};
    Vector2D disp = {1.0, 2.0};
    Point2D sum = tail + disp;
    
    REQUIRE_THAT( sum.x, Catch::Matchers::WithinAbs(2.0, tol) );
    REQUIRE_THAT( sum.y, Catch::Matchers::WithinAbs(4.0, tol) );
}

TEST_CASE( "Sub Operator", "[operator-]" )
{
    Point2D tail2 = {1.0, 2.0};
    Point2D head = {1.0, 2.0};
    Vector2D sum2 = tail2 - head;
    
    REQUIRE_THAT( sum2.x, Catch::Matchers::WithinAbs(0, tol) );
    REQUIRE_THAT( sum2.y, Catch::Matchers::WithinAbs(0, tol) );
}

TEST_CASE("operator+","[Vector2D]")
{
    Vector2D v1 = Vector2D{4, 2};
    Vector2D v2 = Vector2D{3, 6};
    Vector2D v3 = v1 + v2;

    REQUIRE_THAT(v3.x, Catch::Matchers::WithinAbs(7.0, tol));
    REQUIRE_THAT(v3.y, Catch::Matchers::WithinAbs(8.0, tol));
}

TEST_CASE("operator-","[Vector2D]")
{
    Vector2D v1 = Vector2D{20, 12};
    Vector2D v2 = Vector2D{8, 6};
    Vector2D v3 = v1 - v2;

    REQUIRE_THAT(v3.x, Catch::Matchers::WithinAbs(12.0, tol));
    REQUIRE_THAT(v3.y, Catch::Matchers::WithinAbs(6.0, tol));
}

TEST_CASE("operator*","[Vector2D]")
{
    Vector2D v1 = Vector2D{1, 2};
    double s = 4.0;
    Vector2D v2 = v1 * s;

    REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(4.0, tol));
    REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(8.0, tol));
}

TEST_CASE("dot", "[Vector2D]")
{
    Vector2D v1 = Vector2D{2, 4};
    Vector2D v2 = Vector2D{1, 2};

    REQUIRE_THAT(dot(v1,v2), Catch::Matchers::WithinAbs(10.0, tol));
}

TEST_CASE("magnitude", "[Vector2D]")
{
    Vector2D v1 = Vector2D{1, 2};
    Vector2D v2 = Vector2D{2, 3};
    Vector2D v3 = Vector2D{3, 4};

    REQUIRE_THAT(magnitude(v1), Catch::Matchers::WithinAbs(2.236, tol));
    REQUIRE_THAT(magnitude(v2), Catch::Matchers::WithinAbs(3.6055, tol));
    REQUIRE_THAT(magnitude(v3), Catch::Matchers::WithinAbs(5.0, tol));
}

TEST_CASE("angle", "[Vector2D]")
{
    Vector2D v1 = Vector2D{1, 2};
    Vector2D v2 = Vector2D{2, 4};
    Vector2D v3 = Vector2D{5, 10};
    Vector2D v4 = Vector2D{-5, 2};

    REQUIRE_THAT(angle(v1,v2), Catch::Matchers::WithinAbs(0, tol));
    REQUIRE_THAT(angle(v3,v4), Catch::Matchers::WithinAbs(-1.654, tol));
}

TEST_CASE("operator+=","[Vector2D]")
{
    Vector2D v1 = Vector2D{1,2};
    Vector2D v2 = v1 + v1;

    REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(2.0, tol));
    REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(4.0, tol));
}

TEST_CASE("operator-=","[Vector2D]")
{
    Vector2D v1 = Vector2D{9,2};
    Vector2D v2 = v1 - v1;

    REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(0, tol));
    REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(0, tol));
}

// TEST_CASE("operator*=","[Vector2D]")
// {
//     Vector2D v1 = Vector2D{9,2};
//     Vector2D v2;

//     REQUIRE_THAT(v2.x, Catch::Matchers::WithinAbs(2.0, tol));
//     REQUIRE_THAT(v2.y, Catch::Matchers::WithinAbs(4.0, tol));
// }