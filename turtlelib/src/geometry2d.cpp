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

    // normalize a Vector2D
    Vector2D normalize(Vector2D v){
        Vector2D normV;
        double m = sqrt((v.x*v.x) + (v.y*v.y));
        normV.x = v.x/m;
        normV.y = v.y/m;
        return normV;
    }

    // add two separate vectors together
    Vector2D operator+(const Vector2D v1, const Vector2D v2){
        Vector2D v3 = v1 + v2;
        return v3;
    }

    // subtract two separate vectors together
    Vector2D operator-(const Vector2D v1, const Vector2D v2){
        Vector2D v3 = v2 - v1;
        return v3;
    }

    // multiplying two vectors together
    Vector2D operator*(const Vector2D v1, const Vector2D v2){
        Vector2D v3 = v1*v2;
        return v3;
    }

    // dot product of vectors
    double dot(const Vector2D v1, const Vector2D v2){
        double x1 = v1.x;
        double y1 = v1.y;
        double x2 = v2.x;
        double y2 = v2.y;

        double dot = x1*x2 + y1*y2;
        return dot;
    }

    // magnitude of a vector
    double magnitude(const Vector2D v){
        double x = v.x;
        double y = v.y;
        double x_s = pow(x,2);
        double y_s = pow(y,2);
        double m = sqrt(x_s + y_s);

        return m;
    }

    // angle betweeen two vectors
    double angle(const Vector2D v1, const Vector2D v2){
        double x1 = v1.x;
        double y1 = v1.y;
        double x2 = v2.x;
        double y2 = v2.y;
        double x1_s = pow(x1,2);
        double y1_s = pow(y1,2);
        double x2_s = pow(x2,2);
        double y2_s = pow(y2,2);
        
        double ang = acos((x1*x2 + y1*y2)/(sqrt(x1_s+y1_s)+sqrt(x2_s+y2_s)));

        return ang;
    }

    


}

