
classdef TestUtilsAdditiveNoiseMeasurementModel < TestUtilsMeasurementModels
    % Provides test utilities for the AdditiveNoiseMeasurementModel class.
    
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
    
    methods (Access = 'protected')
        function [initState, measModel, measurements,  ...
                  trueStateMean, trueStateCov, ...
                  trueMeasMean, trueMeasCov, trueCrossCov] = updateConfig(obj, stateDecomp, multiMeas)
            initState = Gaussian(obj.initMean, obj.initCov);
            
            measModel = AddNoiseMeasModel(stateDecomp);
            measModel.setNoise(obj.measNoise);
            
            if stateDecomp
                mat = [measModel.measMatrix zeros(3, 1)];
            else
                mat = measModel.measMatrix;
            end
            
            [noiseMean, noiseCov] = obj.measNoise.getMeanAndCovariance();
            
            if multiMeas
                measurements = obj.twoMeas;
                
                trueMeasMean   = mat * obj.initMean + noiseMean;
                trueMeasMean   = repmat(trueMeasMean, 2, 1);
                trueMeasCov    = mat * obj.initCov * mat';
                trueMeasCov    = [trueMeasCov + noiseCov trueMeasCov
                                  trueMeasCov            trueMeasCov + noiseCov];
                invTrueMeasCov = trueMeasCov \ eye(6);
                crossCov       = obj.initCov * mat';
                crossCov       = [crossCov crossCov];
            else
                measurements = obj.singleMeas;
                
                trueMeasMean   = mat * obj.initMean + noiseMean;
                trueMeasCov    = mat * obj.initCov * mat' + noiseCov;
                invTrueMeasCov = trueMeasCov \ eye(3);
                crossCov       = obj.initCov * mat';
            end
            
            K = crossCov * invTrueMeasCov;
            
            trueStateMean = obj.initMean + K * (measurements(:) - trueMeasMean);
            trueStateCov  = obj.initCov - K * crossCov';
            
            if stateDecomp
                % If state decomposition is enabled, the true
                % cross-covariance matirx is
                trueCrossCov = crossCov(1, :);
            else
                trueCrossCov = crossCov;
            end
        end
    end
    
    properties (Constant, Access = 'private')
        initMean   = [0.3 -pi]';
        initCov    = [0.5 0.1; 0.1 3];
        measNoise  = Gaussian([2 -1 0.5]', [ 2   -0.5 0
                                            -0.5  1.3 0.5
                                             0    0.5 sqrt(2)]);
        singleMeas = [ 1
                      -2
                       5];
        twoMeas    = [ 1  1.5
                      -2 -1.85
                       5 -4   ];
    end
end
