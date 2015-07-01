
classdef AnalyticSystemModel < handle
    % Abstract base class for analytic system models.
    %
    % AnalyticSystemModel Methods:
    %   analyticPredictedMoments - Analytic calculation of the first two moments of the predicted state distribution.
    
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
        % Analytic calculation of the first two moments of the predicted state distribution.
        %
        % Parameters:
        %   >> stateMean (Column vector)
        %      The current state mean.
        %
        %   >> stateCov (Positive definite matrix)
        %      The current state covariance.
        %
        % Returns:
        %   << predictedStateMean (Column vector)
        %      The predicted state mean.
        %
        %   << predictedStateCov (Positive definite matrix)
        %      The predicted state covariance.
        [predictedStateMean, predictedStateCov] = analyticPredictedMoments(obj, stateMean, stateCov);
    end
end
