
classdef ASIRPF < SIRPF
    % The Auxiliary Sequential Importance Resampling Particle Filter (ASIRPF).
    %
    % Use the step() method to perform the auxiliary version of the SIR-PF (i.e.,
    % a combined time and measurement update). Note that the separated predictions
    % and updates of the SIRPF are still possible by simply using predict() and
    % update(), respectively (e.g., in case where a prediction is required but no
    % measurement is available).
    %
    % Notice: according to [1], the ASIRPF is to be preferred to the SIRPF if the
    % system/process noise is small. In case of large system/process noise, however,
    % the ASIRPF can even degrade the performance.
    %
    % ASIRPF Methods:
    %   ASIRPF                     - Class constructor.
    %   getName                    - Get the filter name / description.
    %   setColor                   - Set the filter color / plotting properties.
    %   getColor                   - Get the current filter color / plotting properties.
    %   setState                   - Set the system state.
    %   getState                   - Get the current system state.
    %   getStateDim                - Get the dimension of the current system state.
    %   predict                    - Perform a time update (prediction step).
    %   update                     - Perform a measurement update (filter step) using the given measurement(s).
    %   step                       - Perform a combined time and measurement update.
    %   getPointEstimate           - Get a point estimate of the current system state.
    %   setNumParticles            - Set the number of particles used by the filter.
    %   getNumParticles            - Get the current number of particles used by the filter.
    %   setMinAllowedNormalizedESS - Set the minimum allowed normalized effective sample size (ESS).
    %   getMinAllowedNormalizedESS - Get the current minimum allowed normalized effective sample size (ESS).
    
    % Literature:
    %   [1] Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
    %       Beyond the Kalman Filter: Particle filters for Tracking Applications,
    %       Artech House Publishers, 2004.
    
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
        function obj = ASIRPF(name)
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
            %      Default name: 'Auxiliary SIR-PF'.
            %
            % Returns:
            %   << obj (ASIRPF)
            %      A new ASIRPF instance.
            
            if nargin < 1
                name = 'Auxiliary SIR-PF';
            end
            
            % Call superclass constructor
            obj = obj@SIRPF(name);
        end
    end
    
    methods (Access = 'protected')
        function performStep(obj, sysModel, measModel, measurements)
            % Implements the combined time and measurement update described in:
            %   Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
            %   Beyond the Kalman Filter: Particle filters for Tracking Applications,
            %   Artech House Publishers, 2004,
            %   Section 3.5.2
            
            if Checks.isClass(sysModel, 'SystemModel')
                predictParticles = @obj.predictParticlesArbitraryNoise;
            elseif Checks.isClass(sysModel, 'AdditiveNoiseSystemModel')
                predictParticles = @obj.predictParticlesAdditiveNoise;
            elseif Checks.isClass(sysModel, 'MixedNoiseSystemModel')
                predictParticles = @obj.predictParticlesMixedNoise;
            else
                obj.errorSysModel('SystemModel', ...
                                  'AdditiveNoiseSystemModel', ...
                                  'MixedNoiseSystemModel');
            end
            
            if ~Checks.isClass(measModel, 'Likelihood')
                obj.errorMeasModel('Likelihood');
            end
            
            % First, resample if necessary
            obj.resampleByESS();
            
            % Predict particles from last time step
            predictedParticles = predictParticles(sysModel, obj.particles, obj.numParticles);
            
            % Compute likelihood values at predicted particles
            predLogValues = measModel.logLikelihood(predictedParticles, measurements);
            
            obj.checkLogLikelihoodEvaluations(predLogValues, obj.numParticles);
            
            % For numerical stability
            predLogValues = predLogValues - max(predLogValues);
            
            values = exp(predLogValues);
            
            % Compute new particle weights
            weights = obj.weights .* values;
            
            % Normalize particle weights
            sumWeights = sum(weights);
            
            if sumWeights <= 0
                obj.ignoreMeas('Sum of computed particle weights is not positive.');
            end
            
            weights = weights / sumWeights;
            
            % Resample particles from last time step with new weights and also save their indices
            [particles, idx] = Utils.systematicResampling(obj.particles, cumsum(weights), obj.numParticles);
            
            % Predict resampled particles
            predictedParticles = predictParticles(sysModel, particles, obj.numParticles);
            
            % Evaluate logaritmic likelihood at newly predicted particles
            logValues = measModel.logLikelihood(predictedParticles, measurements);
            
            obj.checkLogLikelihoodEvaluations(logValues, obj.numParticles);
            
            % Compute posterior particle weights using the previously computed log-likelihood values
            logValues = logValues - predLogValues(idx);
            
            % For numerical stability
            logValues = logValues - max(logValues);
            
            weights = exp(logValues);
            
            % Normalize posterior particle weights
            sumWeights = sum(weights);
            
            if sumWeights <= 0
                obj.ignoreMeas('Sum of computed posterior particle weights is not positive.');
            end
            
            % Save new particle weights
            obj.weights = weights / sumWeights;
            
            % Save new particles
            obj.particles = predictedParticles;
        end
    end
end
