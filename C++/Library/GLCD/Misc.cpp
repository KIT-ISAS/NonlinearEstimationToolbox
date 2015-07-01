
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

#include <GLCD/Misc.h>
#include <random>
    
namespace GLCD {

void meanCorrection(const Eigen::MatrixXd& samples,
                    Eigen::MatrixXd& correctedSamples)
{
    const Eigen::MatrixXd::Index numSamples = samples.cols();
    const double weight = 1.0 / numSamples;
    
    const Eigen::VectorXd mean = weight * samples.rowwise().sum();
    
    correctedSamples = samples.colwise() - mean;
}

void covarianceCorrection(const Eigen::MatrixXd& samples,
                          Eigen::MatrixXd& correctedSamples)
{
    const Eigen::MatrixXd::Index numSamples = samples.cols();
    const double weight = 1.0 / numSamples;
    
    const Eigen::MatrixXd cov = weight * (samples * samples.transpose());
    
    correctedSamples = cov.inverse().llt().matrixL().transpose() * samples;
}

void stdNormalRndMatrix(unsigned int rows,
                        unsigned int cols,
                        Eigen::MatrixXd& rndMatrix)
{
    rndMatrix.resize(rows, cols);
    
    std::random_device rd;
    std::default_random_engine gen(rd());
    std::normal_distribution<double> dist(0, 1);
    
    for (unsigned int i = 0; i < rows; i++) {
        for (unsigned int j = 0; j < cols; j++) {
            rndMatrix(i, j) = dist(gen);
        }
    }
}

double normalizedCovError(const Eigen::MatrixXd& samples)
{
    const Eigen::MatrixXd::Index numSamples = samples.cols();
    const Eigen::MatrixXd::Index dim = samples.rows();
    
    const Eigen::MatrixXd cov = (samples * samples.transpose()) / (double)numSamples;
    
    const double covError = (cov - Eigen::MatrixXd::Identity(dim, dim)).norm() / (dim * dim);
    
    return covError;
}

}   // namespace GLCD
