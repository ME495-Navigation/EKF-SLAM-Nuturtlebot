#include <catch2/catch_test_macros.hpp> // Include Catch2
#include "turtlelib/svg.hpp"

TEST_CASE("Svg class generates correct SVG content", "[Svg]") {
    turtlelib::Svg svg;

    SECTION("Draw a point") {
        svg.dPoint({50, 50}, "blue");
        REQUIRE(svg.footer().str() == "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n"
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
                                      "</defs><circle cx=\"50\" cy=\"50\" r=\"2\" fill=\"blue\"/>\n"
                                      "</svg>");
    }

}