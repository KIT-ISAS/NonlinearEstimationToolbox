
classdef AdditiveNoiseMeasurementModel < Likelihood
    % Abstract base class for measurement models corrupted by additive noise.
    %
    % AdditiveNoiseMeasurementModel Methods:
    %   setNoise            - Set the measurement noise.
    %   measurementEquation - The measurement equation.
    %   logLikelihood       - Evaluate the logarithmic likelihood function.
    %   derivative          - Compute the first-order and second-order derivatives of the implemented measurement equation.
    %   simulate            - Simulate a measurement for the given system state.
    
    % >> This function/class is part of the Nonlinear Estimation Toolbox
    %
    %    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
    %
    %    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
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
            % Set the measurement noise.
            %
            % Parameters:
            %   >> noise (Subclass of Distribution)
            %      The new measurement noise.
            
            if Checks.isClass(noise, 'Distribution')
                obj.noise = noise;
            else
                error('AdditiveNoiseMeasurementModel:InvalidNoise', ...
                      'noise must be a subclass of Distribution.');
            end
        end
        
        function logValues = logLikelihood(obj, stateSamples, measurement)
            % Evaluate the logarithmic likelihood function.
            %
            % Parameters:
            %   >> stateSamples (Matrix)
            %      L column-wise arranged state samples.
            %
            %   >> measurement (Column vector)
            %      The measurement vector.
            %
            % Returns:
            %   << logValues (Row vector)
            %      L column-wise arranged logarithmic likelihood function values.
            
            if ~Checks.isColVec(measurement)
                obj.error('InvalidMeasurement', ...
                          'measurement must be a column vector.');
            end
            
            dimNoise   = obj.noise.getDim();
            dimMeas    = size(measurement, 1);
            numSamples = size(stateSamples, 2);
            
            if dimMeas ~= dimNoise
                error('AdditiveNoiseMeasurementModel:InvalidMeasurementNoise', ...
                      'Measurement and additive measurement noise with different dimensions.');
            end
            
            % Evaluate deterministic measurement equation
            deterministicMeas = obj.measurementEquation(stateSamples);
            
            % Check computed deterministic measurements
            if ~Checks.isMat(deterministicMeas, dimMeas, numSamples)
                error('AdditiveNoiseMeasurementModel:InvalidMeasurements', ...
                      ['Computed deterministic measurements have to be ' ...
                       'stored as a matrix of dimension %dx%d.'], ...
                       dimMeas, numSamples);
            end
            
            values = bsxfun(@minus, measurement, deterministicMeas);
            
            logValues = obj.noise.logPdf(values);
        end
        
        function [stateJacobian, stateHessians] = derivative(obj, nominalState)
            % Compute the first-order and second-order derivatives of the implemented measurement equation.
            %
            % By default, the derivatives are computed using difference quotients.
            %
            % Mainly used by EKF and EKF2.
            %
            % Parameters:
            %   >> nominalState (Column vector)
            %      The nominal system state vector.
            %
            % Returns:
            %   << stateJacobian (Square matrix)
            %      The Jacobian of the state variables.
            %
            %   << stateHessians (3D matrix)
            %      The Hessians of the state variables.
            
            if nargout == 1
                stateJacobian = Utils.diffQuotientState(@obj.measurementEquation, nominalState);
            else
                [stateJacobian, ...
                 stateHessians] = Utils.diffQuotientState(@obj.measurementEquation, nominalState);
            end
        end
        
        function measurement = simulate(obj, state)
            % Simulate a measurement for the given system state.
            %
            % Parameters:
            %   >> state (Column vector)
            %      The system state.
            %
            % Returns:
            %   << measurement (Column vector)
            %      The simulated measurement.
            
            if ~Checks.isColVec(state)
                error('AdditiveNoiseMeasurementModel:InvalidSystemState', ...
                      'state must be a column vector.');
            end
            
            noiseSamples = obj.noise.drawRndSamples(1);
            
            measurement = obj.measurementEquation(state) + noiseSamples;
        end
    end
    
    methods (Abstract)
        % The measurement equation.
        %
        % Parameters:
        %   >> stateSamples (Matrix)
        %      L column-wise arranged state samples.
        %
        % Returns:
        %   << measurements (Matrix)
        %      L column-wise arranged measurement samples.
        measurements = measurementEquation(obj, stateSamples);
    end
    
    properties (SetAccess = 'private', GetAccess = 'public')
        % The measurement noise.
        noise;
    end
end
