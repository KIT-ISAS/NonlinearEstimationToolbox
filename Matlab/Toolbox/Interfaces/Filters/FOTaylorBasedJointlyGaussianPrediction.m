
classdef FOTaylorBasedJointlyGaussianPrediction < GaussianFilter
    % Abstract base class for Gaussian filters that use a first-order Taylor based prediction,
    % where the joint density of state and noise is assumed to be Gaussian.
    %
    % FOTaylorBasedJointlyGaussianPrediction Methods:
    %   FOTaylorBasedJointlyGaussianPrediction - Class constructor.
    %   copy                                   - Copy a Filter instance.
    %   copyWithName                           - Copy a Filter instance and give the copy a new name / description.
    %   getName                                - Get the filter name / description.
    %   setColor                               - Set the filter color / plotting properties.
    %   getColor                               - Get the current filter color / plotting properties.
    %   setState                               - Set the system state.
    %   getState                               - Get the current system state.
    %   getStateDim                            - Get the dimension of the current system state.
    %   predict                                - Perform a time update (prediction step).
    %   update                                 - Perform a measurement update (filter step) using the given measurement(s).
    %   step                                   - Perform a combined time and measurement update.
    %   getPointEstimate                       - Get a point estimate of the current system state.
    %   setUseAnalyticSystemModel              - Enable or disable the use of analytic moment calculation during a prediction.
    %   getUseAnalyticSystemModel              - Get the current use of analytic moment calculation during a prediction.
    %   setStateDecompDim                      - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim                      - Get the dimension of the unobservable part of the system state.
    
    % Literature:
    %   Dan Simon,
    %   Optimal State Estimation,
    %   1st ed. Wiley & Sons, 2006.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2017  Jannik Steinbring <jannik.steinbring@kit.edu>
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
    
    methods
        function obj = FOTaylorBasedJointlyGaussianPrediction(name)
            % Class constructor.
            %
            % Parameters:
            %   >> name (Char)
            %      An appropriate filter name / description of the implemented
            %      filter. The Filter subclass should set this during its
            %      construction to a meaningful default value (e.g., 'EKF'),
            %      or the user should specify an appropriate name (e.g., 
            %      'PF (10k Particles)').
            %
            % Returns:
            %   << obj (FOTaylorBasedJointlyGaussianPrediction)
            %      A new FOTaylorBasedJointlyGaussianPrediction instance.
            
            % Call superclass constructor
            obj = obj@GaussianFilter(name);
        end
    end
    
    methods (Access = 'protected')
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsArbitraryNoise(obj, sysModel)
            [noiseMean, ~, noiseCovSqrt] = sysModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            % Linearize system model around current state mean and noise mean
            [stateJacobian, ...
             noiseJacobian] = sysModel.derivative(obj.stateMean, noiseMean);
            
            % Check computed derivatives
            obj.checkStateJacobian(stateJacobian, obj.dimState, obj.dimState);
            obj.checkNoiseJacobian(noiseJacobian, obj.dimState, dimNoise);
            
            % Compute predicted state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean, noiseMean);
            
            % Compute predicted state covariance
            A = stateJacobian * obj.stateCovSqrt;
            B = noiseJacobian * noiseCovSqrt;
            
            predictedStateCov = A * A' + B * B';
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsAdditiveNoise(obj, sysModel)
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Linearize system model around current state mean
            stateJacobian = sysModel.derivative(obj.stateMean);
            
            % Check computed derivative
            obj.checkStateJacobian(stateJacobian, obj.dimState, obj.dimState);
            
            % Compute predicted state mean
            predictedStateMean = sysModel.systemEquation(obj.stateMean) + noiseMean;
            
            % Compute predicted state covariance
            A = stateJacobian * obj.stateCovSqrt;
            
            predictedStateCov = A * A' + noiseCov;
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsMixedNoise(obj, sysModel)
            [addNoiseMean, addNoiseCov] = sysModel.additiveNoise.getMeanAndCovariance();
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimAddNoise);
            
            [mean, cov] = obj.predictedMomentsArbitraryNoise(sysModel);
            
            % Compute predicted state mean
            predictedStateMean = mean + addNoiseMean;
            
            % Compute predicted state covariance
            predictedStateCov = cov + addNoiseCov;
        end
    end
end
