#include "turtlelib/diff_drive.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib
{
    DiffDrive::DiffDrive(double wheel_radius, double track_width)
    {
        radius = wheel_radius;
        width = track_width;
    }

    DiffDrive::DiffDrive(double wheel_radius, double track_width, Transform2D s)
    {
        radius = wheel_radius;
        width = track_width;
        q = s;
    }

    Transform2D DiffDrive::get_q() const
    {
        return q;
    }

    WheelAng DiffDrive::get_phi() const
    {
        return phi;
    }

    Twist2D DiffDrive::get_body_twist(const double new_left, const double new_right){
        // Velocity from new positions
        // Refer to the second page of doc/Kinematics.pdf, step 1
        WheelAng vel = {new_right - phi.right_ang, new_left - phi.left_ang};

        // Twist for calculations
        // Refer to the second page of doc/Kinematics.pdf, step 2
        return {
            radius/(width*2.0) * (vel.right_ang - vel.left_ang),
            radius/2.0 * (vel.right_ang + vel.left_ang),
            0.0
        };
    }

    Twist2D DiffDrive::f_kin(double right_ang_new, double left_ang_new)
    {
        // // state deltas
        // // phi_delta, x_delta, y_delta = Modern Robotics Eqn 13.15
        // const double phi_delta = (radius/width)*(-left_ang_new + right_ang_new);
        // const double x_delta = (0.5*radius)*cos(q.rotation());
        // const double y_delta = (0.5*radius)*sin(q.rotation());

        // // update wheel state
        // double phi_new = normalize_angle(phi_delta + q.rotation());
        // phi.right_ang = right_ang_new;
        // phi.left_ang = left_ang_new;

        // // update q
        // q = Transform2D({q.translation().x + x_delta, q.translation().y + y_delta}, phi_new);

        // // return twist
        // return Twist2D{phi_delta, x_delta, 0.0};

        Twist2D tw = get_body_twist(left_ang_new, right_ang_new);
        
        // Update wheel angles
        phi = {right_ang_new, left_ang_new};
    

        // Transform needed to get to new wheel positions
        // Refer to the second page of doc/Kinematics.pdf, step 3
        Transform2D t = integrate_twist(tw);
        
        // Transform from world to base
        Transform2D t_wb = {{q.translation().x, q.translation().y}, q.rotation()};

        // Calculate transform to new location of the robot
        // Refer to the second page of doc/Kinematics.pdf, step 4
        Transform2D t_wb_new = t_wb * t;

        // Update config based on calculations
        q = {
            {t_wb_new.translation().x,
            t_wb_new.translation().y},
            t_wb_new.rotation()
        };

        return tw;
    }

    WheelAng DiffDrive::i_kin(Twist2D t)
    {
        // need to add logic for slipping wheels
        if(almost_equal(t.y, 0.0))
        {
            const double phi_right = (t.x + width*t.omega)/radius;
            const double phi_left = (t.x - width*t.omega)/radius;
            return WheelAng{phi_right, phi_left};
        }
        else
        {
            throw std::logic_error("Wheels slipping because y component of twist is not zero!");
        }
    }

    void DiffDrive::q_new(Transform2D s)
    {
        q = s;
    }
}