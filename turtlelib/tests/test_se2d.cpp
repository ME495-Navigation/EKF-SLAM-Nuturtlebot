#include "turtlelib/se2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

using namespace turtlelib;

    TEST_CASE("Input Twist2D check","[Twist2D]") // Ishani Narwankar
    {
        // checking for stringstream twist input 
        std::stringstream check{""};
        Twist2D t = {1.5, 3.05, 4.87};
        check << t;

        // set requirement
        REQUIRE(check.str() == "[1.5 3.05 4.87]");
    }

    TEST_CASE("Input Transform2D check","[Transform2D]") // Ishani Narwankar
    {
        // checking for stringstream transform input
        std::stringstream check{""};
        Transform2D t = {Vector2D{1.5, 3.05}, deg2rad(90)};
        check << t;

        // set requirement
        REQUIRE_THAT(t.rotation(), Catch::Matchers::WithinAbs(deg2rad(90), 1.0E-3));
        REQUIRE_THAT(t.translation().x, Catch::Matchers::WithinAbs(1.5, 1.0E-3));
        REQUIRE_THAT(t.translation().y, Catch::Matchers::WithinAbs(3.05, 1.0E-3));
    }

    TEST_CASE("Output Twist2D check", "[Twist2D]") // Ishani Narwankar
    {
        // checking for stringstream twist output
        std::stringstream check;
        Twist2D t = {1.5, 3.05, 1.57};

        check.str ("[90 1.5 3.05]");
        check >> t;

        // set requirement
        REQUIRE(t.omega == 90.0);
        REQUIRE(t.x == 1.5);
        REQUIRE(t.y == 3.05);
        REQUIRE_THAT(t.omega, Catch::Matchers::WithinAbs(90.0, 1.0E-3));
        REQUIRE_THAT(t.x, Catch::Matchers::WithinAbs(1.5, 1.0E-3));
        REQUIRE_THAT(t.y, Catch::Matchers::WithinAbs(3.05, 1.0E-3));
    }

    TEST_CASE("Output Transform2D check", "[Transform2D]") // Ishani Narwankar
    {
        // checking for stringstream transfrom output
        std::stringstream check;
        Transform2D t;
        check << "90 1.5 3.05";
        check >> t;

        // set requirement
        REQUIRE_THAT(t.rotation(), Catch::Matchers::WithinRel(deg2rad(90)));
        REQUIRE_THAT(t.translation().x, Catch::Matchers::WithinRel(1.5));
        REQUIRE_THAT(t.translation().y, Catch::Matchers::WithinRel(3.05));
    }

    TEST_CASE("Twist2D>>", "[se2d]") { // Max Palay
        std::stringstream ss("0.1 1.43 -0.2\n"); 
        Twist2D twist;
        ss >> twist;
        REQUIRE_THAT(twist.omega, Catch::Matchers::WithinAbs(0.1, 1.0E-8));
        REQUIRE_THAT(twist.x, Catch::Matchers::WithinAbs(1.43, 1.0E-8));
        REQUIRE_THAT(twist.y, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
        ss = std::stringstream("[0.1 1.43 -0.2]\n"); 
        twist = Twist2D{};
        ss >> twist;
        REQUIRE_THAT(twist.omega, Catch::Matchers::WithinAbs(0.1, 1.0E-8));
        REQUIRE_THAT(twist.x, Catch::Matchers::WithinAbs(1.43, 1.0E-8));
        REQUIRE_THAT(twist.y, Catch::Matchers::WithinAbs(-0.2, 1.0E-8));
    }

    TEST_CASE("Initializing transform", "[Transform2D]") { // Kyle Wang
        Transform2D tf1{0.32}; // can't use tf1 = {...} because explicit, can't use copy-list-initialization
        Transform2D tf2{Vector2D{3.1, -1.3}};
        Transform2D tf3 = {Vector2D{3.1, -1.3}, 0.32};
        Transform2D tf4;

        REQUIRE_THAT(tf1.rotation(), Catch::Matchers::WithinRel(0.32));
        REQUIRE_THAT(tf1.translation().x, Catch::Matchers::WithinRel(0.0));
        REQUIRE_THAT(tf1.translation().y, Catch::Matchers::WithinRel(0.0));

        REQUIRE_THAT(tf2.rotation(), Catch::Matchers::WithinRel(0.0));
        REQUIRE_THAT(tf2.translation().x, Catch::Matchers::WithinRel(3.1));
        REQUIRE_THAT(tf2.translation().y, Catch::Matchers::WithinRel(-1.3));

        REQUIRE_THAT(tf3.rotation(), Catch::Matchers::WithinRel(0.32));
        REQUIRE_THAT(tf3.translation().x, Catch::Matchers::WithinRel(3.1));
        REQUIRE_THAT(tf3.translation().y, Catch::Matchers::WithinRel(-1.3));

        REQUIRE_THAT(tf4.rotation(), Catch::Matchers::WithinRel(0.0));
        REQUIRE_THAT(tf4.translation().x, Catch::Matchers::WithinRel(0.0));
        REQUIRE_THAT(tf4.translation().y, Catch::Matchers::WithinRel(0.0));
    }

    TEST_CASE("Transform2D(point)", "[se2d]") { // Max Palay
        // identity
        Vector2D trans{0.0, 0.0};
        Transform2D tf{trans, 0.0};
        Point2D a{0.3, -0.1};
        Point2D res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.3, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.1, 1.0E-4));

        // pure translation
        trans = {0.2, -0.2};
        tf = Transform2D(trans, 0.0);
        a = {0.3, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.5, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.3, 1.0E-4));

        // pure rotation
        trans = {0.0, 0.0};
        tf = Transform2D(trans, 0.2);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.1179, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.0781, 1.0E-4));

        // rotation & translation
        trans = {0.1, -0.1};
        tf = Transform2D(trans, 3.8);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(-0.0403, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.0821, 1.0E-4));
    }

    TEST_CASE("Transform2D(vector)", "[se2d]") { // Max Palay
        // identity
        Vector2D trans{0.0, 0.0};
        Transform2D tf{trans, 0.0};
        Vector2D a{0.3, -0.1};
        Vector2D res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.3, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.1, 1.0E-4));

        // pure translation
        trans = {0.2, -0.2};
        tf = Transform2D(trans, 0.0);
        a = {0.3, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.3, 1.0E-4));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.1, 1.0E-4));

        // pure rotation
        trans = {0.0, 0.0};
        tf = Transform2D(trans, 0.2);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(0.1179, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(-0.0781, 1.0E-4));

        // rotation & translation
        trans = {0.1, -0.1};
        tf = Transform2D(trans, 3.8);
        a = {0.1, -0.1};
        res = tf(a);
        REQUIRE_THAT(res.x, Catch::Matchers::WithinAbs(-0.1403, 1.0E-3));
        REQUIRE_THAT(res.y, Catch::Matchers::WithinAbs(0.0179, 1.0E-4));
    }

    TEST_CASE("Transforming a twist", "[se2d]") { // Kyle Wang
        Transform2D tf = {Vector2D{0.2, 1.1}, 0.34};
        Twist2D tw = {1.0, 0.707, 0.707};
        Twist2D ans = tf(tw);
        REQUIRE_THAT(ans.omega, Catch::Matchers::WithinRel(1.0));
        REQUIRE_THAT(ans.x, Catch::Matchers::WithinRel(1.531, 0.001));
        REQUIRE_THAT(ans.y, Catch::Matchers::WithinRel(0.702, 0.001));
    }

    TEST_CASE("Inverting a transform", "[transform2D]") { // Kyle Wang
        Transform2D tf = {Vector2D{-1.2, 0.35}, 0.29};
        Transform2D tfInv = tf.inv();
        REQUIRE_THAT(tfInv.rotation(), Catch::Matchers::WithinRel(-0.29));
        REQUIRE_THAT(tfInv.translation().x, Catch::Matchers::WithinRel(1.050, 0.001));
        REQUIRE_THAT(tfInv.translation().y, Catch::Matchers::WithinRel(-0.679, 0.001));
    }

    TEST_CASE("Transform2D*=", "[se2d]") { // Max Palay
        // try composition with the inverse
        Vector2D trans{1.2, -2.2};
        Transform2D tf{trans, 0.6};
        Transform2D inv;
        inv = tf.inv();
        tf *= inv;
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0E-3));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0E-3));

        // try composition with another regular tf
        Vector2D trans1{1.2, -2.2};
        Transform2D tf1{trans1, 0.6};
        Vector2D trans2{0.3, 4.1};
        Transform2D tf2{trans2, -0.1};
        tf1 *= tf2;
        REQUIRE_THAT(tf1.rotation(), Catch::Matchers::WithinAbs(0.5, 1.0E-3));
        REQUIRE_THAT(tf1.translation().x, Catch::Matchers::WithinAbs(-0.867, 1.0E-3));
        REQUIRE_THAT(tf1.translation().y, Catch::Matchers::WithinAbs(1.353, 1.0E-3));
    }

    TEST_CASE("Obtain transform components", "[Transform2D]"){ // Kyle Wang
        Transform2D tf = {Vector2D{2.1, -4.3}, 2.3};
        REQUIRE_THAT(tf.rotation(), Catch::Matchers::WithinRel(2.3));
        REQUIRE_THAT(tf.translation().x, Catch::Matchers::WithinRel(2.1));
        REQUIRE_THAT(tf.translation().y, Catch::Matchers::WithinRel(-4.3));
    }

    TEST_CASE("Multiply operator *", "[Transform2D]"){ // Kyle Wang
        Transform2D tf1 = {Vector2D{1.2, -2.2}, 0.6};
        Transform2D tf2 = {Vector2D{0.3, 4.1}, -0.1};
        Transform2D tf3 = tf1 * tf2;
        REQUIRE_THAT(tf3.rotation(), Catch::Matchers::WithinRel(0.5));
        REQUIRE_THAT(tf3.translation().x, Catch::Matchers::WithinRel(-0.867, 0.001));
        REQUIRE_THAT(tf3.translation().y, Catch::Matchers::WithinRel(1.353, 0.001));
    }

    TEST_CASE("integrate_twist", "[Twist2D/Transform2D]"){ // Ishani Narwankar
        // define test twists for each case
        Twist2D tw1;
        Twist2D tw2;
        Twist2D tw3;

        // pure translation
        tw1.x = 1;
        tw1.y = 0;
        tw1.omega = 0;

        // pure rotation
        tw2.x = 0;
        tw2.y = 0;
        tw2.omega = PI/2;

        // translation + rotation
        tw3.x = 1;
        tw3.y = 1;
        tw3.omega = PI/2;

        // calculate integrate_twist for each case
        Transform2D a1;
        Transform2D a2;
        Transform2D a3;

        a1 = integrate_twist(tw1);
        a2 = integrate_twist(tw2);
        a3 = integrate_twist(tw3);

        // test cases
        REQUIRE_THAT(a1.translation().x, Catch::Matchers::WithinAbs(1,1.0E-3));
        REQUIRE_THAT(a1.translation().y, Catch::Matchers::WithinAbs(0,1.0E-3));
        REQUIRE_THAT(a1.rotation(), Catch::Matchers::WithinAbs(0,1.0E-3));

        REQUIRE_THAT(a2.translation().x, Catch::Matchers::WithinAbs(0,1.0E-3));
        REQUIRE_THAT(a2.translation().y, Catch::Matchers::WithinAbs(0,1.0E-3));
        REQUIRE_THAT(a2.rotation(), Catch::Matchers::WithinAbs(PI/2,1.0E-3));

        REQUIRE_THAT(a3.translation().x, Catch::Matchers::WithinAbs(0.0,1.0E-3));
        REQUIRE_THAT(a3.translation().y, Catch::Matchers::WithinAbs(1.273,1.0E-3));
        REQUIRE_THAT(a3.rotation(), Catch::Matchers::WithinAbs(PI/2,1.0E-3));

    }
