#include "turtlelib/diff_drive.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace turtlelib{
    // robot moves forward - f_kin and i_kin
    TEST_CASE("Robot moves forward", "[diff_drive]"){
        // create a robot
        DiffDrive robot(1.0, 1.0);
        
        auto s = robot.get_q();
        auto phi = robot.get_phi();
        REQUIRE(s.translation().x == 0.0);
        REQUIRE(s.translation().y == 0.0);
        REQUIRE(s.rotation() == 0.0);
        REQUIRE(phi.left_ang == 0.0);
        REQUIRE(phi.right_ang == 0.0);

        // move the robot forward by x = 2.0
        Twist2D t = Twist2D{0.0, 2.0, 0.0};

        // test i_kin
        WheelAng w = robot.i_kin(t);
        REQUIRE_THAT(w.left_ang, Catch::Matchers::WithinAbs(1.0, 1e-6));
        REQUIRE_THAT(w.right_ang, Catch::Matchers::WithinAbs(1.0, 1e-6));

        // test f_kin
        robot.f_kin(w.right_ang, w.left_ang);
        REQUIRE_THAT(robot.get_q().translation().x, Catch::Matchers::WithinAbs(2.0, 1e-6));
        REQUIRE_THAT(robot.get_q().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(robot.get_q().rotation(), Catch::Matchers::WithinAbs(0.0, 1e-6));
    }

    // robot executes a pure rotation - f_kin and i_kin
    TEST_CASE("Robot executes pure rotation", "[diff_drive]"){
        // create a robot
        DiffDrive robot(1.0, 1.0);
        
        auto s = robot.get_q();
        auto phi = robot.get_phi();
        REQUIRE(s.translation().x == 0.0);
        REQUIRE(s.translation().y == 0.0);
        REQUIRE(s.rotation() == 0.0);
        REQUIRE(phi.left_ang == 0.0);
        REQUIRE(phi.right_ang == 0.0);

        // rotate the robot by pi/2
        Twist2D t = Twist2D{PI/2, 0.0, 0.0};

        // test i_kin
        WheelAng w = robot.i_kin(t);
        REQUIRE_THAT(w.left_ang, Catch::Matchers::WithinAbs(-0.5*1.0*PI/2, 1e-6));
        REQUIRE_THAT(w.right_ang, Catch::Matchers::WithinAbs(0.5*1.0*PI/2, 1e-6));

        // test f_kin
        robot.f_kin(w.right_ang, w.left_ang);
        REQUIRE_THAT(robot.get_q().translation().x, Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(robot.get_q().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(robot.get_q().rotation(), Catch::Matchers::WithinAbs(PI/2, 1e-6));
    }

    // robot follows the arc of a circle - f_kin and i_kin
    TEST_CASE("Robot follows arc of a circle", "[diff_drive]"){
        // create a robot
        DiffDrive robot(1.0, 1.0);
        
        auto s = robot.get_q();
        auto phi = robot.get_phi();
        REQUIRE(s.translation().x == 0.0);
        REQUIRE(s.translation().y == 0.0);
        REQUIRE(s.rotation() == 0.0);
        REQUIRE(phi.left_ang == 0.0);
        REQUIRE(phi.right_ang == 0.0);

        // move the robot forward by x = 2.0
        Twist2D t = Twist2D{PI/2, 2.0, 0.0};

        // test i_kin
        WheelAng w = robot.i_kin(t);
        REQUIRE_THAT(w.left_ang, Catch::Matchers::WithinAbs(2.0-0.5*PI/2, 1e-6));
        REQUIRE_THAT(w.right_ang, Catch::Matchers::WithinAbs(2.0+0.5*PI/2, 1e-6));

        // test f_kin
        robot.f_kin(w.right_ang, w.left_ang);
        REQUIRE_THAT(robot.get_q().translation().x, Catch::Matchers::WithinAbs(2.0, 1e-6));
        REQUIRE_THAT(robot.get_q().translation().y, Catch::Matchers::WithinAbs(0.0, 1e-6));
        REQUIRE_THAT(robot.get_q().rotation(), Catch::Matchers::WithinAbs(PI/2, 1e-6));
    }

    // an impossible twist to follow is provided
    TEST_CASE("Impossible twist", "[diff_drive]"){
        // create a robot
        DiffDrive robot(1.0, 1.0);

        // impossible twist
        Twist2D t = Twist2D{PI, 0.0, 2.0};
        CHECK_THROWS(robot.i_kin(t));
    }


}