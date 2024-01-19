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