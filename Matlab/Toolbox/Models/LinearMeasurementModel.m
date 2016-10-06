
classdef LinearMeasurementModel < AdditiveNoiseMeasurementModel & AnalyticMeasurementModel
    % Linear measurement model corrupted by additive noise.
    %
    % LinearMeasurementModel Methods:
    %   LinearMeasurementModel     - Class constructor.
    %   setNoise                   - Set the measurement noise.
    %   measurementEquation        - The measurement equation.
    %   logLikelihood              - Evaluate the logarithmic likelihood function of the implemented measurement equation.
    %   derivative                 - Compute the first-order and second-order derivatives of the implemented measurement equation.
    %   simulate                   - Simulate one ore more measurements for a given system state.
    %   analyticMeasurementMoments - Analytic calculation of the first two moments of the measurement distribution.
    %   setMeasurementMatrix       - Set the measurement matrix.
    
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
    
    methods
        function obj = LinearMeasurementModel(measMatrix)
            % Class constructor.
            %
            % Parameters:
            %   >> measMatrix (Matrix or empty matrix)
            %      The measurement matrix.
            %      An empty matrix means the identity matrix of appropriate dimensions.
            %      Default: Empty matrix.
            %
            % Returns:
            %   << obj (LinearMeasurementModel)
            %      A new LinearMeasurementModel instance.
            
            if nargin < 1
                obj.setMeasurementMatrix([]);
            else
                obj.setMeasurementMatrix(measMatrix);
            end
        end
        
        function setMeasurementMatrix(obj, measMatrix)
            % Set the measurement matrix.
            %
            % By default, the measurement matrix is an empty matrix
            % (i.e., an identity matrix of appropriate dimensions).
            %
            % Parameters:
            %   >> measMatrix (Matrix or empty matrix)
            %      The new measurement matrix.
            %      An empty matrix means the identity matrix of appropriate dimensions.
            
            if ~Checks.isMat(measMatrix) && ...
               ~isempty(measMatrix)
                error('LinearMeasurementModel:InvalidMeasurementMatrix', ...
                      'measMatrix must be a matrix.');
            end
            
            obj.measMatrix = measMatrix;
        end
    end
    
    methods (Sealed)
        function [stateJacobian, stateHessians] = derivative(obj, nominalState)
            dimState = size(nominalState, 1);
            
            if isempty(obj.measMatrix)
                stateJacobian = eye(dimState);
                stateHessians = zeros(dimState, dimState, dimState);
            else
                dimMeas = obj.checkMeasMatrix(dimState);
                
                stateJacobian = obj.measMatrix;
                stateHessians = zeros(dimState, dimState, dimMeas);
            end
        end
        
        function measurements = measurementEquation(obj, stateSamples)
            if isempty(obj.measMatrix)
                measurements = stateSamples;
            else
                dimState = size(stateSamples, 1);
                
                obj.checkMeasMatrix(dimState);
                
                measurements = obj.measMatrix * stateSamples;
            end
        end
        
        function [measMean, measCov, ...
                  stateMeasCrossCov] = analyticMeasurementMoments(obj, stateMean, stateCov, numMeas)
            [noiseMean, noiseCov] = obj.noise.getMeanAndCovariance();
            dimNoise = size(noiseMean, 1);
            dimState = size(stateMean, 1);
            
            if isempty(obj.measMatrix)
                obj.checkMeasNoise(dimState, dimNoise);
                
                % Measurement mean
                measMean = stateMean + noiseMean;
                
                % Measurement covariance
                measCov = stateCov;
                
                % State measurement cross-covariance
                stateMeasCrossCov = stateCov;
            else
                obj.checkMeasMatrix(dimState, dimNoise);
                
                % Measurement mean
                measMean = obj.measMatrix * stateMean + noiseMean;
                
                % Measurement covariance
                measCov = obj.measMatrix * stateCov * obj.measMatrix';
                
                % State measurement cross-covariance
                stateMeasCrossCov = stateCov * obj.measMatrix';
            end
            
            measMean          = repmat(measMean, numMeas, 1);
            measCov           = Utils.baseBlockDiag(measCov, noiseCov, numMeas);
            stateMeasCrossCov = repmat(stateMeasCrossCov, 1, numMeas);
        end
    end
    
    methods (Access = 'private')
        function dimMeasMatrixMeas = checkMeasMatrix(obj, dimState, dimNoise)
            [dimMeasMatrixMeas, dimMeasMatrixState] = size(obj.measMatrix);
            
            if dimState ~= dimMeasMatrixState
                error('LinearMeasurementModel:IncompatibleMeasurementMatrix', ...
                      'System state and measurement matrix with incompatible dimensions.');
            end
            
            if nargin == 3
                if dimNoise ~= dimMeasMatrixMeas
                    error('LinearMeasurementModel:IncompatibleMeasurementMatrix', ...
                          'Measurement noise and measurement matrix with incompatible dimensions.');
                end
            end
        end
        
        function checkMeasNoise(~, dimState, dimNoise)
            if dimNoise ~= dimState
                error('LinearMeasurementModel:IncompatibleMeasurementNoise', ...
                      'System state and measurement noise with incompatible dimensions.');
            end
        end
    end
    
    properties (SetAccess = 'private', GetAccess = 'public')
        % The measurement matrix.
        measMatrix;
    end
end
