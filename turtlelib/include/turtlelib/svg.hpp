#ifndef TURTLELIB_SVG_HPP_INCLUDE_GUARD
#define TURTLELIB_SVG_HPP_INCLUDE_GUARD
/// \file
/// \brief Drawing vectors in svg.

#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include<sstream>
#include<string>

namespace turtlelib{

    class Svg
    {
    public:
        Svg()
        {
            // Input header into string
            svgSave << header;
        }


        // Draw point
        void dPoint(Point2D a, const std::string& color);

        // Draw vector
        void dVec(Point2D head, Point2D tail, const std::string& color);

        // Draw coordinate frame
        void dCoordFrame(Point2D xl, Point2D yl, Point2D c, const std::string& color);

        // Ouput svgSave
        std::stringstream & footer();

    private:
        // save svg info into string
        std::stringstream svgSave;

        // Header of svg file
        std::string header = "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n"
                             "<defs>\n"
                              "<marker>\n"
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