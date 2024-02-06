
#ifndef TURTLELIB_SVG_HPP_INCLUDE_GUARD
#define TURTLELIB_SVG_HPP_INCLUDE_GUARD
/// \file
/// \brief Drawing vectors in svg.

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include<sstream>
#include<string>

namespace turtlelib{
    /// \brief Draws points, vectors, and coordinate frames and saves to .svg
    class Svg
    {
    public:
        Svg()
        {
            // Input header into string
            svgSave << header;
        }

        ///\brief draw point
        ///\param a Point2D in format [x,y]
        ///\param color string input for color choice
        void dPoint(Point2D a, const std::string& color);

        ///\brief draw vector
        ///\param head-Point2D in for head of vector
        ///\param tail-Point2D for tail of vector
        ///\param color string input for color choice
        void dVec(Point2D head, Point2D tail, const std::string& color);

        ///\brief draw coordinate frame
        ///\param origin Point2D for origin of coordinate frame
        ///\param x Point2D for x axis
        ///\param y Point2D for y axis
        ///\param tloc Point2D for location of frame label
        ///\param name string for frame label
        void dCoordFrame(Point2D origin, Point2D x, Point2D y, Point2D tloc, std::string name); // const std::strintg &

        /// \brief output footer and save svg to file
        // 1. the return value is not documented
        // 2. this is dangerous, you are returning a reference to an internal class varaible, which allows
        // external entities to access the private member
        // does calling  maintain the validity of this object
        std::stringstream & footer();


    private:
        ///\brief save svg to stringstream
        std::stringstream svgSave;

        ///\brief include constant info for svg header
        // static constexpr
        std::string header = "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n"
                             "<defs>\n"
                              "<marker\n"
                                "style=\"overflow:visible\"\n"
                                "id=\"Arrow1Sstart\"\n"
                                "refX=\"0.0\"\n"
                                "refY=\"0.0\"\n"
                                "orient=\"auto\">\n"
                                "<path\n"
                                    "transform=\"scale(0.2) translate(6,0)\"\n"
                                    "style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n"
                                    "d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"\n"
                                    "/>\n"
                                "</marker>\n"
                            "</defs>";

        
    };

}






#endif
