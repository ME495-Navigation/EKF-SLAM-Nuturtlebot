#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <sstream>
#include <fstream>
#include <istream>
#include <iostream>

    int main()
    {
        // create an object for the output file
        turtlelib::Svg file;

        // prompt the user to enter two transforms
        std::cout << "Enter transform T_{a,b}:" << std::endl;
        turtlelib::Transform2D Tab;
        std::cin >> Tab;

        turtlelib::Transform2D Tbc;
        std::cout << "Enter transform T_{b,c}:" << std::endl;
        std::cin >> Tbc;

        // compute and output Tab, Tba, Tbc, Tcb, Tac, and Tca
        std::cout << "T_{a,b} = " << Tab << std::endl;
        turtlelib::Transform2D Tba = Tab.inv();
        std::cout << "T_{b,a} = " << Tba << std::endl;

        std::cout << "T_{b,c} = " << Tbc << std::endl;
        turtlelib::Transform2D Tcb = Tbc.inv();
        std::cout << "T_{c,b} = " << Tcb << std::endl;

        turtlelib::Transform2D Tac = Tab*Tbc;
        std::cout << "T_{a,c} = " << Tac << std::endl;

        turtlelib::Transform2D Tca = Tac.inv();
        std::cout << "T_{c,a} = " << Tca << std::endl;

        // draw each frame in svg file format with frame {a} at (0,0)


        // prompt the user to enter a point p_a in Frame {a}
        std::cout << "Enter point p_a:" << std::endl;
        turtlelib::Point2D pa;
        std::cin >> pa;

        // compute pa's location in b and in c
        std::cout << "p_a: " << pa << std::endl;
        turtlelib::Point2D pb = Tba(pa);
        std::cout << "p_b: " << pb << std::endl;
        turtlelib::Point2D pc = Tca(pa);
        std::cout << "p_c: " << pc << std::endl;

        // draw these points
        file.dPoint(pa,"purple");
        file.dPoint(pb,"brown");
        file.dPoint(pc,"orange");

        // prompt the user to enter a vector v_b
        std::cout << "Enter vector v_b:" << std::endl;
        turtlelib::Vector2D vb;
        std::cin >> vb;
        // normalize the vector to form vbhat
        turtlelib::Vector2D v_bhat = turtlelib::normalize(vb);
        std::cout << "v_bhat: " << v_bhat << std::endl;
        turtlelib::Vector2D va = Tab(vb);
        std::cout << "v_a: " << va << std::endl;
        std::cout << "v_b: " << vb << std::endl;
        turtlelib::Vector2D vc = Tcb(vb);
        std::cout << "v_c: " << vc << std::endl;

        // prompt the user to enter in a twist V_b
        std::cout << "Enter twist V_b:" << std::endl;
        turtlelib::Twist2D Vb;
        std::cin >> Vb;

        // calculata Va and Vc
        turtlelib::Twist2D Va = Tab(Vb);
        std::cout << "V_a: " << Va << std::endl;
        std::cout << "V_b: " << Vb << std::endl;
        turtlelib::Twist2D Vc = Tcb(Vb);
        std::cout << "V_c: " << Vc << std::endl;

        std::ofstream outputFile("/tmp/frames.svg");
        if (outputFile.is_open()){
            outputFile << file.footer().rdbuf();
            outputFile.close();
            std::cout << "SVG content saved to /tmp/frames.svg" << std::endl;
        } else {
            std::cerr << "Error writing to file." << std::endl;
        }

        return 0;
    }
