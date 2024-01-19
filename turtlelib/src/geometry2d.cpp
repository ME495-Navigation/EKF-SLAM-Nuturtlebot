// Defining and implementing functions in geometry2d.cpp

#include "turtlelib/geometry2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib
{
    //  implementing functions in order of appearance in .hpp
    // keeping rad in range -pi to pi
    double normalize_angle(double rad){
        while(rad > PI)
        {
            rad -= 2*PI;
        }
        while(rad <= -PI)
        {
            rad += 2*PI;
        }
        return rad;
    }

    // get x and y components from point p
    std::ostream & operator<<(std::ostream & os, const Point2D & p){
        os << "[" << p.x << ", " << p.y << "]";
        return os;
    }

    // set x and y components for point p
    std::istream & operator>>(std::istream & is, Point2D & p){
        is >> p.x >> p.y;
        return is;
    }

    // calculate vector that points from p1 to p2 (p2-p1)
    Vector2D operator-(const Point2D & head, const Point2D & tail){
        Vector2D vec;
        vec.x = tail.x - head.x;
        vec.y = tail.y - head.y;
        return vec;
    }

    // add vector to point to create new point displaced by vector
    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        Point2D dispP;
        dispP.x = tail.x + disp.x;
        dispP.y = tail.y + disp.y;
        return dispP;
    }

    // output a vector in format [x y]
    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << ", " << v.y << "]";
        return os;
    }

    // take input of a vector in format [x y]
    std::istream & operator>>(std::istream & is, Vector2D & v){
        is >> v.x >> v.y;
        return is;
    }
}

