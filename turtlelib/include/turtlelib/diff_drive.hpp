#ifndef TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
#define TURTLELIB_DIFF_DRIVE_HPP_INCLUDE_GUARD
/// \file
/// \brief Tracks the position of a robot's wheels, implements forward and inverse kinematics

#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

namespace turtlelib
{
    /// \brief robot wheel states (angles and velocities)
    struct WheelAng
    {
        double right_ang = 0.0;
        double left_ang = 0.0;
    };

    /// \brief update the state of the turtlebot
    class DiffDrive
    {
        private:
            /// \brief wheel params to be set later with params.yaml
            double width = 1.0;
            double radius = 1.0;

            /// \brief current wheel angles
            WheelAng phi;
            /// \brief current q
            Transform2D q;

        public:
            /// \brief constructor
            DiffDrive(double wheel_radius, double track_width);

            /// \brief world to body frame
            DiffDrive(double wheel_radius, double track_width, Transform2D s);

            /// \brief current q in world frame
            Transform2D get_q() const;

            /// \brief current wheel state
            WheelAng get_phi() const;

            /// \brief compute twist in body frame
            Twist2D get_body_twist(const double new_left, const double new_right);

            /// \brief forward kinematics - update q based on curr wheel state
            Twist2D f_kin(double right_ang_new, double left_ang_new);

            /// \brief inverse kinematics - compute wheel velocities required to make the robot move at a given twist
            WheelAng i_kin(Twist2D t);
           
            /// \brief change q to desired x,y,theta
            void q_new(Transform2D s);
            
    };       
}

#endif