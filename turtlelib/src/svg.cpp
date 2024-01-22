#include "turtlelib/svg.hpp"

namespace turtlelib
{
    void Svg::dPoint(Point2D a, const std::string& color)
    {
        svgSave << "<circle cx=\"" << a.x << "\" cy=\"" << a.y << "\" r=\"2\" fill=\"" << color << "\"/>\n";
    }

    void Svg::dVec(Point2D head, Point2D tail, const std::string& color)
    {
        svgSave << "<line tailx=\"" << tail.x << "\" taily=\"" << tail.y <<
                              "\" headx=\"" << head.x << "\" heady=\"" << head.y <<
                              "\" stroke=\"" << color << "\" stroke-width=\"2\"/>\n";
    }

    void Svg::dCoordFrame(Point2D xl, Point2D yl, Point2D c, const std::string& color)
    {
        svgSave << "<line xlx=\"" << xl.x << "\" xly=\"" << xl.y <<
                              "\" cx=\"" << c.x << "\" cy=\"" << c.y <<
                              "\" ylx=\"" << yl.x << "\" yly=\"" << yl.y <<
                              "\" stroke=\"" << color << "\" stroke-width=\"2\"/>\n";
    }

    std::stringstream & Svg::footer()
    {
        svgSave << "</svg>";
        return svgSave;
    }
}


