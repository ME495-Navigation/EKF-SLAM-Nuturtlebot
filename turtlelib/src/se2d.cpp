#include "turtlelib/se2d.hpp"
#include <iostream>
#include <cmath>

namespace turtlelib
{
    //  implementing functions in order of appearance in .hpp
    // returns the ostream with the twist data inserted
    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        os << "[" << tw.omega << " " << " " << tw.x << " " << tw.y << "]";
        return os;
    }

    // returns the istream with the twist characters removed
    std::istream & operator>>(std::istream & is, Twist2D & tw){
        char b;
        if(is >> b && b == '[')
        {
            is >> tw.omega >> tw.x >> tw.y >> b;
        }
        else
        {
            is >> tw.omega >> tw.x >> tw.y;
        }
        return is;
    }

    // create an identity transformation
    Transform2D::Transform2D(){
        x = 0.0;
        y = 0.0;
        w = 0.0;
    }
    
    // create a transformation that is pure translation
    Transform2D::Transform2D(Vector2D trans){
        x = trans.x;
        y = trans.y;
        w = 0.0;
    }

    // transformation with pure rotation
    Transform2D::Transform2D(double radians){
        w = radians;
        x = 0.0;
        y = 0.0;
    }

    // transformation with translation and rotation
    Transform2D::Transform2D(Vector2D trans, double radians){
        x = trans.x;
        y = trans.y;
        w = radians;
    }

    // apply transformation to a 2D point
    Point2D Transform2D::operator()(Point2D p) const{
        Point2D newP;
        newP.x = ((p.x*std::cos(w))-(p.y*std::sin(w)));
        newP.y = ((p.x*std::sin(w))-(p.y*std::cos(w)));
    }

    // apply transformation to 2D vector
    Vector2D Transform2D::operator()(Vector2D v) const{
        Vector2D newV;
        newV.x = (v.x*std::cos(w)) - (v.y*std::sin(w)) + x;
        newV.y = (v.x*std::sin(w)) - (v.y*std::cos(w)) + y;
    }

    // apply transformation to 2D Twist
    Twist2D Transform2D::operator()(Twist2D v) const{
        Twist2D newV2;
        newV2.x = v.x;
        newV2.y = v.y + (y*v.omega);
        newV2.omega = (-x*v.y) + (std::cos(w)*v.omega) + (x*v.x);
        return newV2;

    }
    
    // invert the transformation
    Transform2D Transform2D::inv() const{
        Transform2D newT;
        newT.w = -w;
        newT.x = (-x*std::cos(w)) - (y*std::sin(w));
        newT.y = (x*std::sin(w)) - (y*std::cos(w));
        return newT;
    }

    // reference the transformed operator
    Transform2D & Transform2D::operator*=(const Transform2D & rhs){

        const auto n_w = w + rhs.w;
        const auto n_x = (rhs.x*std::cos(w)) - (rhs.y*std::sin(w)) + x;
        const auto n_y = (rhs.x*std::sin(w)) + (rhs.y*std::cos(w)) + y;

        x = n_x;
        y = n_y;
        w = n_w;

        return *this;
    }

    // return the x, y translation
    Vector2D Transform2D::translation() const{
        Vector2D t;
        t.x = x;
        t.y = y;
        
        return t;
    }

    // return angular displacement (w) of transfrom
    double Transform2D::rotation() const{
        return w;
    }

    // print out transform like deg:_, x:_, y:_
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << tf.w << " x: " << tf.x << " y: " << tf.y;
        return os;
    }

    // read a transform from stdin
    std::istream & operator>>(std::istream & is, Transform2D & tf){
        Vector2D t = tf.translation();
        double w = tf.rotation();
        is >> w >> t.x >> t.y;
        return is;
    }

    // multiply two transforms and return result
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        lhs *= rhs;

        return lhs;
    }
}