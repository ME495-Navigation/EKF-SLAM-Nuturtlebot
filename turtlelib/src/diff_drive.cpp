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

    void DiffDrive::f_kin(double right_ang_new, double left_ang_new)
    {
        // state deltas
        // phi_delta, x_delta, y_delta = Modern Robotics Eqn 13.15
        const double phi_delta = (radius/width)*(-left_ang_new + right_ang_new);
        const double x_delta = (0.5*radius)*cos(q.rotation());
        const double y_delta = (0.5*radius)*sin(q.rotation());

        // update wheel state
        double phi_new = normalize_angle(phi_delta + q.rotation());
        phi.right_ang = right_ang_new;
        phi.left_ang = left_ang_new;

        // update q
        q = Transform2D({q.translation().x + x_delta, q.translation().y + y_delta}, phi_new);
    }

    auto DiffDrive::i_kin(Twist2D t)
    {
        // need to add logic for slipping wheels
        if(almost_equal(t.y, 0.0, 1e-6))
        {
            const double phi_right = (1/radius)*(t.x + ((width*t.omega)/2));
            const double phi_left = (1/radius)*(t.x - ((width*t.omega)/2));
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