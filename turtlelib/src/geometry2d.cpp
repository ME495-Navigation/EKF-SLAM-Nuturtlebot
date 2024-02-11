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
        return {tail.x - head.x, tail.y - head.y};
    }

    // add vector to point to create new point displaced by vector
    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        return {tail.x + disp.x, tail.y + disp.y};
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
        const auto & m = sqrt((v.x*v.x) + (v.y*v.y));
        return {v.x/m, v.y/m};
    }

    // adding the same vector to itself
    Vector2D & Vector2D::operator+=(const Vector2D v1){
        x += v1.x;
        y += v1.y;

        return *this;
    }

    // add two separate vectors together
    Vector2D operator+(Vector2D v1, const Vector2D & v2){
        return v1+=v2;
    }

    // subtracting the same vector from itself
    Vector2D & Vector2D::operator-=(const Vector2D v1){
        x -= v1.x;
        y -= v1.y;

        return *this;
    }

    // subtract two separate vectors together
    Vector2D operator-(Vector2D v1, const Vector2D & v2){
        return v1-=v2;
    }

    // multiplying the same vector to itself
    Vector2D & Vector2D::operator*=(const double s){
        x *= s;
        y *= s;

        return *this;
    }

    // multiplying two vectors together
    Vector2D operator*(Vector2D v1, const double & s){
        return v1*=s;
    }

    // dot product of vectors
    double dot(Vector2D v1, Vector2D v2){
        double x1 = v1.x;
        double y1 = v1.y;
        double x2 = v2.x;
        double y2 = v2.y;

        double dot = x1*x2 + y1*y2;
        return dot;
    }

    // magnitude of a vector
    double magnitude(Vector2D v){
        double x = v.x;
        double y = v.y;
        double x_s = pow(x,2);
        double y_s = pow(y,2);
        double m = sqrt(x_s + y_s);

        return m;
    }

    // angle betweeen two vectors
    double angle(Vector2D v1, Vector2D v2){
        double x1 = v1.x;
        double y1 = v1.y;
        double x2 = v2.x;
        double y2 = v2.y;
        
        double ang = atan2(y1,x1) - atan2(y2,x2);

        return ang;
    }

    


}

