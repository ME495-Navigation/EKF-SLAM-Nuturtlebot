#include "turtlelib/svg.hpp"

namespace turtlelib
{
    void Svg::dPoint(Point2D a, const std::string& color)
    {
        // convert from svg coords to pixel coords 
        double scale = 96.0;
        double x_off = 408.0;
        double y_off = 528.0;
        double x = (a.x*scale) + x_off;
        double y = (-a.y*scale) + y_off;
        svgSave << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"2\" fill=\"" << color << "\"/>\n";
    }

    void Svg::dVec(Point2D head, Point2D tail, const std::string& color)
    {
        // convert from svg coords to pixel coords
        double scale = 96.0;
        double x_off = 408.0;
        double y_off = 528.0;
        double x1 = (head.x*scale) + x_off;
        double y1 = (-head.y*scale) + y_off;
        double x2 = (tail.x*scale) + x_off;
        double y2 = (-tail.y*scale) + y_off;
        svgSave << "<line x1=\"" << x1 << "\" y1=\"" << y1 <<
                              "\" x2=\"" << x2 << "\" y2=\"" << y2 <<
                              "\" stroke=\"" << color << "\" stroke-width=\"2\" "" marker-start=\"url(#Arrow1Sstart)\" />\n";
    }

    void Svg::dCoordFrame(Point2D origin, Point2D x, Point2D y, Point2D tloc, std::string name)
    {
        const std::string& colorx = "red";
        const std::string& colory = "green";
        dVec(x,origin,colorx);
        dVec(y,origin,colory);
        double scale = 96.0;
        double x_off = 408.0;
        double y_off = 528.0;
        double tlocx = (tloc.x*scale) + x_off;
        double tlocy = (-tloc.y*scale) + y_off;
        svgSave << "<text x=\"" << tlocx << "\" y=\"" << tlocy << "\">" << "{" << name << "} </text>\n";
    }

    std::stringstream & Svg::footer()
    {
        svgSave << "</svg>";
        return svgSave;
    }
}


