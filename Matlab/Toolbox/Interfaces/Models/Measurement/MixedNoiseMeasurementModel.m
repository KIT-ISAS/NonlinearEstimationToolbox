
classdef MixedNoiseMeasurementModel < handle
    % Abstract base class for system models corrupted by a mixture additive and arbitrary noise.
    %
    % MixedNoiseMeasurementModel Methods:
    %   setNoise            - Set the non-additive measurement noise.
    %   setAdditiveNoise    - Set the additive measurement noise.
    %   measurementEquation - The measurement equation.
    %   derivative          - Compute the derivative of the implemented measurement equation.
    %   simulate            - Simulate one ore more measurements for a given system state.
    
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
        function setNoise(obj, noise)
            % Set the non-additive measurement noise.
            %
            % Parameters:
            %   >> noise (Subclass of Distribution or cell array containing subclasses of Distribution)
            %      The new non-additive measurement noise.
            
            if Checks.isClass(noise, 'Distribution')
                obj.noise = noise;
            else
                obj.noise = JointDistribution(noise);
            end
        end
        
        function setAdditiveNoise(obj, additiveNoise)
            % Set the additive measurement noise.
            %
            % Parameters:
            %   >> additiveNoise (Subclass of Distribution or cell array containing subclasses of Distribution)
            %      The new additive measurement noise.
            
            if Checks.isClass(additiveNoise, 'Distribution')
                obj.additiveNoise = additiveNoise;
            else
                obj.additiveNoise = JointDistribution(additiveNoise);
            end
        end
        
        function [stateJacobian, ...
                  noiseJacobian] = derivative(obj, nominalState, nominalNoise)
            % Compute the derivative of the implemented measurement equation.
            %
            % By default, the Jacobians are computed using a difference quotient.
            %
            % Mainly used by the EKF.
            %
            % Parameters:
            %   >> nominalState (Column vector)
            %      The nominal system state vector to linearize the measurement equation.
            %
            %   >> nominalNoise (Column vector)
            %      The nominal system noise vector to linearize the measurement equation.
            %
            % Returns:
            %   << stateJacobian (Square matrix)
            %      The Jacobian of the state variables.
            %
            %   << noiseJacobian (Matrix)
            %      The Jacobian of the noise variables.
            
            [stateJacobian, ...
             noiseJacobian] = Utils.diffQuotientStateAndNoise(@(s, n) obj.measurementEquation(s, n), ...
                                                              nominalState, nominalNoise);
        end
        
        function measurements = simulate(obj, state, numMeasurements)
            % Simulate one ore more measurements for a given system state.
            %
            % Parameters:
            %   >> state (Column vector)
            %      The system state.
            %
            %   >> numMeasurements (Positive scalar)
            %      The number of measurements to simulate.
            %      Default: One measurement will be simulated.
            %
            % Returns:
            %   << measurements (Matrix)
            %      Column-wise arranged simulated measurements.
            
            if ~Checks.isColVec(state)
                error('MixedNoiseMeasurementModel:InvalidSystemState', ...
                      'state must be a column vector.');
            end
            
            if nargin < 3
                numMeasurements = 1;
            else
                if ~Checks.isPosScalar(numMeasurements)
                    error('MixedNoiseMeasurementModel:InvalidSystemState', ...
                          'numMeasurements must be a positive scalar.');
                end
                
                numMeasurements = ceil(numMeasurements);
            end
            
            addNoiseSamples = obj.additiveNoise.drawRndSamples(numMeasurements);
            noiseSamples    = obj.noise.drawRndSamples(numMeasurements);
            
            measurements = obj.measurementEquation(repmat(state, 1, numMeasurements), noiseSamples);
            measurements = measurements + addNoiseSamples;
        end
    end
    
    methods (Abstract)
        % The measurement equation.
        %
        % Parameters:
        %   >> stateSamples (Matrix)
        %      L column-wise arranged state samples.
        %
        %   >> noiseSamples (Matrix)
        %      L column-wise arranged measurement noise samples.
        %
        % Returns:
        %   << measurements (Matrix)
        %      L column-wise arranged measurement samples.
        measurements = measurementEquation(obj, stateSamples, noiseSamples);
    end
    
    properties (SetAccess = 'private', GetAccess = 'public')
        % The non-additive measurement noise.
        noise;
        
        % The additive measurement noise.
        additiveNoise;
    end
end
