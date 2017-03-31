//
// Created by Oleg Leyzerov on 30/03/2017.
//

#include "jacobian.h"
#include "Eigen/Dense"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

MatrixXd Jacobian(const VectorXd &z) {
    float px, py, vx, vy, px2, py2, psums, spsums, spsums3;
    px = z[0];
    py = z[1];
    MatrixXd Jac = MatrixXd(3, 4);
    Jac << 0,0,0,0,0,0,0,0,0,0,0,0;
    if (px==0 && py==0) {
        std::cout << "Error: received position's x and y coordinates are equal to zero" << std::endl;
        return Jac;
        //throw std::invalid_argument( "received position's x and y coordinates are equal to zero" );
    } else {
        vx = z[2];
        vy = z[3];
        px2 = px * px;
        py2 = py * py;
        psums = px2 + py2;
        spsums = pow(psums, 0.5);
        spsums3 = pow(spsums, 3);


        Jac << px / spsums, py / spsums, 0, 0,
                -py / psums, px / psums, 0, 0,
                py * (vx * py - vy * px) / spsums3, px * (vy * px - vx * py) / spsums3, px / spsums, py / spsums;
        return Jac;
    }
}