
classdef AnalyticMeasurementModel < handle
    % Abstract base class for analytic measurement models.
    %
    % AnalyticMeasurementModel Methods:
    %   analyticMeasurementMoments - Analytic calculation of the first two moments of the measurement distribution.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2015  Jannik Steinbring <jannik.steinbring@kit.edu>
    %
    %                        Institute for Anthropomatics and Robotics
    %                        Chair for Intelligent Sensor-Actuator-Systems (ISAS)
    %                        Karlsruhe Institute of Technology (KIT), Germany
    %
    %                        http://isas.uka.de
    %
    %    This program is free software: you can redistribute it and/or modify
    %    it under the terms of the GNU General Public License as published by
    %    the Free Software Foundation, either version 3 of the License, or
    %    (at your option) any later version.
    %
    %    This program is distributed in the hope that it will be useful,
    %    but WITHOUT ANY WARRANTY; without even the implied warranty of
    %    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    %    GNU General Public License for more details.
    %
    %    You should have received a copy of the GNU General Public License
    %    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    
    methods (Abstract)
        % Analytic calculation of the first two moments of the measurement distribution.
        %
        % Parameters:
        %   >> stateMean (Column vector)
        %      The current state mean.
        %
        %   >> stateCov (Positive definite matrix)
        %      The current state covariance.
        %
        %   >> numMeas (Positive scalar)
        %      Number of measurements passed to the Filter.update() function.
        %
        % Returns:
        %   << measMean (Column vector)
        %      The measurement mean vector.
        %
        %   << measCov (Positive definite matrix)
        %      The measurement covariance matrix.
        %
        %   << stateMeasCrossCov (Matrix)
        %      The state measurement cross-covariance matrix.
        [measMean, measCov, stateMeasCrossCov] = analyticMeasurementMoments(obj, stateMean, stateCov, numMeas);
    end
end
