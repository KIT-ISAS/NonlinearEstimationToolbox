
classdef ASIRPF < SIRPF
    % The auxiliary sampling importance resampling particle filter (ASIRPF).
    %
    % Use the step() method to perform the auxiliary version of the SIRPF (i.e.,
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
    %   copy                       - Copy a Filter instance.
    %   copyWithName               - Copy a Filter instance and give the copy a new name/description.
    %   getName                    - Get the filter name/description.
    %   setColor                   - Set the filter color/plotting properties.
    %   getColor                   - Get the filter color/plotting properties.
    %   setState                   - Set the system state.
    %   getState                   - Get the system state.
    %   setStateMeanAndCov         - Set the system state by means of mean and covariance matrix.
    %   getStateMeanAndCov         - Get mean and covariance matrix of the system state.
    %   getStateDim                - Get the dimension of the system state.
    %   predict                    - Perform a state prediction.
    %   update                     - Perform a measurement update.
    %   step                       - Perform a combined state prediction and measurement update.
    %   setNumParticles            - Set the number of particles used by the filter.
    %   getNumParticles            - Get the number of particles used by the filter.
    %   setMinAllowedNormalizedESS - Set the minimum allowed normalized effective sample size (ESS).
    %   getMinAllowedNormalizedESS - Get the minimum allowed normalized effective sample size (ESS).
    
    % Literature:
    %   [1] Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
    %       Beyond the Kalman Filter: Particle filters for Tracking Applications,
    %       Artech House Publishers, 2004.
    
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
            %      Default name: 'ASIRPF'.
            %
            % Returns:
            %   << obj (ASIRPF)
            %      A new ASIRPF instance.
            
            if nargin < 1
                name = 'ASIRPF';
            end
            
            % Call superclass constructor
            obj = obj@SIRPF(name);
        end
    end
    
    methods (Access = 'protected')
        function performStep(obj, sysModel, measModel, measurement)
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
            predLogValues = measModel.logLikelihood(predictedParticles, measurement);
            
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
            logValues = measModel.logLikelihood(predictedParticles, measurement);
            
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
