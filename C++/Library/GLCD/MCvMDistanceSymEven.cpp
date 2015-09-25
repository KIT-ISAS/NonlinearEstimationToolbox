
/* >> This file is part of the Nonlinear Estimation Toolbox
 *
 *    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
 *
 *    Copyright (C) 2015  Jannik Steinbring <jannik.steinbring@kit.edu>
 *                        Martin Pander <martin.pander@student.kit.edu>
 *
 *                        Institute for Anthropomatics and Robotics
 *                        Chair for Intelligent Sensor-Actuator-Systems (ISAS)
 *                        Karlsruhe Institute of Technology (KIT), Germany
 *
 *                        http://isas.uka.de
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <GLCD/MCvMDistanceSymEven.h>
#include <cmath>

namespace GLCD {

MCvMDistanceSymEven::MCvMDistanceSymEven(unsigned int d,
                                         unsigned int n) :
    MCvMDistanceSym (d, n, 1.0 / (2.0 * n))
{

}

MCvMDistanceSymEven::~MCvMDistanceSymEven()
{
    
}

Eigen::MatrixXd MCvMDistanceSymEven::getSamples() const
{
    Eigen::MatrixXd samples(dim, 2 * numHalfSamples);
    
    unsigned int j = 0;
    
    for (unsigned int i = 0; i < numHalfSamples; ++i) {
        samples.col(j++) =  halfSamples.col(i);
        samples.col(j++) = -halfSamples.col(i);
    }
    
    return samples;
}

void MCvMDistanceSymEven::computeD2(double& D2)
{
    MCvMDistanceSym::computeD2Base(D2);
}

void MCvMDistanceSymEven::computeD3(double& D3)
{
    MCvMDistanceSym::computeD3Base(D3);
}

void MCvMDistanceSymEven::computeGrad2(Eigen::MatrixXd& grad2)
{
    MCvMDistanceSym::computeGrad2Base(grad2);
}

}   // namespace GLCD

