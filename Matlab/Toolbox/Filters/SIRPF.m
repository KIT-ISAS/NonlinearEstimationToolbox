
classdef SIRPF < ParticleFilter
    % The sampling importance resampling particle filter (SIRPF).
    %
    % SIRPF Methods:
    %   SIRPF                      - Class constructor.
    %   copy                       - Copy a Filter instance.
    %   copyWithName               - Copy a Filter instance and give the copy a new name/description.
    %   getName                    - Get the filter name/description.
    %   setColor                   - Set the filter color/plotting properties.
    %   getColor                   - Get the filter color/plotting properties.
    %   setState                   - Set the system state.
    %   getState                   - Get the system state.
    %   getStateDim                - Get the dimension of the system state.
    %   getStateMeanAndCov         - Get mean and covariance matrix of the system state.
    %   predict                    - Perform a state prediction.
    %   update                     - Perform a measurement update.
    %   step                       - Perform a combined state prediction and measurement update.
    %   setNumParticles            - Set the number of particles used by the filter.
    %   getNumParticles            - Get the number of particles used by the filter.
    %   setMinAllowedNormalizedESS - Set the minimum allowed normalized effective sample size (ESS).
    %   getMinAllowedNormalizedESS - Get the minimum allowed normalized effective sample size (ESS).
    
    % Literature:
    %   Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
    %   Beyond the Kalman Filter: Particle filters for Tracking Applications,
    %   Artech House Publishers, 2004.
    
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
        function obj = SIRPF(name)
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
            %      Default name: 'SIRPF'.
            %
            % Returns:
            %   << obj (SIRPF)
            %      A new SIRPF instance.
            
            if nargin < 1
                name = 'SIRPF';
            end
            
            % Call superclass constructor
            obj = obj@ParticleFilter(name);
            
            obj.particles    = [];
            obj.weights      = [];
            obj.numParticles = 1000;
            
            obj.minAllowedNormalizedESS = 0.5;
        end
        
        function state = getState(obj)
            state = DiracMixture(obj.particles, obj.weights);
        end
        
        function [stateMean, stateCov] = getStateMeanAndCov(obj)
            if nargout == 1
                stateMean = Utils.getMeanAndCov(obj.particles, obj.weights);
            else
                [stateMean, stateCov] = Utils.getMeanAndCov(obj.particles, obj.weights);
            end
        end
        
        function setNumParticles(obj, numParticles)
            % Set the number of particles used by the filter.
            %
            % By default, 1000 particles are used.
            %
            % Parameters:
            %   >> numParticles (Positive scalar)
            %      The number of particles used by the filter.
            
            if ~Checks.isPosScalar(numParticles)
                obj.error('InvalidNumberOfParticles', ...
                          'numParticles must be a positive scalar.');
            end
            
            numParticles = ceil(numParticles);
            
            if isempty(obj.particles)
                obj.numParticles = numParticles;
            elseif numParticles ~= obj.numParticles
                obj.resample(numParticles);
            end
        end
        
        function numParticles = getNumParticles(obj)
            % Get the number of particles used by the filter.
            %
            % Returns:
            %   << numParticles (Positive scalar)
            %      The number of particles used by the filter.
            
            numParticles = obj.numParticles;
        end
        
        function setMinAllowedNormalizedESS(obj, minAllowedNormalizedESS)
            % Set the minimum allowed normalized effective sample size (ESS).
            %
            % Determines how often a resampling will be performed.
            %
            % By default, a normalized ESS of 0.5 will be used.
            %
            % Parameters:
            %   >> minAllowedNormalizedESS (Scalar)
            %      The minimum allowed noramlized ESS between 0 and 1.
            
            if ~(Checks.isScalar(minAllowedNormalizedESS) && ...
                 minAllowedNormalizedESS >= 0 && ...
                 minAllowedNormalizedESS <= 1)
                obj.error('InvalidNoramlizedESS', ...
                          'minAllowedNormalizedESS must be a scalar in [0, 1].');
            end
            
            obj.minAllowedNormalizedESS = minAllowedNormalizedESS;
        end
        
        function minAllowedNormalizedESS = getMinAllowedNormalizedESS(obj)
            % Get the minimum allowed normalized effective sample size (ESS).
            %
            % Returns:
            %   << minAllowedNormalizedESS (Scalar)
            %      The minimum allowed noramlized ESS between 0 and 1.
            
            minAllowedNormalizedESS = obj.minAllowedNormalizedESS;
        end
    end
    
    methods (Access = 'protected')
        function performSetState(obj, state)
            if Checks.isClass(state, 'DiracMixture')
                % Directly use the Dirac mixture components as system state
                [obj.particles, obj.weights] = state.getComponents();
                
                % We also have to update the number of particles used by the particle filter
                obj.numParticles = state.getNumComponents();
            else
                % Draw random particles
                obj.particles = state.drawRndSamples(obj.numParticles);
                
                % Particles are equally weighted
                obj.weights = repmat(1 / obj.numParticles, 1, obj.numParticles);
            end
        end
        
        function performPrediction(obj, sysModel)
            % Implements the prediction described in:
            %   Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
            %   Beyond the Kalman Filter: Particle filters for Tracking Applications,
            %   Artech House Publishers, 2004,
            %   Section 3.5.1
            
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
            
            % First, resample if necessary
            obj.resampleByESS();
            
            % Then, perform the state prediction
            obj.particles = predictParticles(sysModel, obj.particles, obj.numParticles);
        end
        
        function performUpdate(obj, measModel, measurement)
            if Checks.isClass(measModel, 'Likelihood')
                obj.updateLikelihood(measModel, measurement);
            else
                obj.errorMeasModel('Likelihood');
            end
        end
        
        function updateLikelihood(obj, measModel, measurement)
            % Implements the measurement update described in:
            %   Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
            %   Beyond the Kalman Filter: Particle filters for Tracking Applications,
            %   Artech House Publishers, 2004,
            %   Section 3.5.1
            
            % First, resample if necessary
            obj.resampleByESS();
            
            % Evaluate likelihood
            values = obj.evaluateLikelihood(measModel, measurement, obj.particles, obj.numParticles);
            
            % Multiply prior weights with likelihood evaluations
            values = values .* obj.weights;
            
            % Normalize particle weights
            sumWeights = sum(values);
            
            if sumWeights <= 0
                obj.ignoreMeas('Sum of computed posterior particle weights is not positive.');
            end
            
            % Save new particle weights
            obj.weights = values / sumWeights;
        end
        
        function resample(obj, numParticles)
            % Perform a resampling.
            %
            % Parameters:
            %   >> numParticles (Positive scalar)
            %      The new number of particles.
            %      Default value: the currently selected number of particles.
            
            if nargin < 2
                % Amount of particles remains the same
                numParticles = obj.numParticles;
            end
            
            % Assumption: Particle weights are always normalized
            cumWeights = cumsum(obj.weights);
            
            % Store new particles
            obj.particles = Utils.systematicResampling(obj.particles, cumWeights, numParticles);
            
            % Store new number of particles
            obj.numParticles = numParticles;
            
            % Equally weighted particles
            obj.weights = repmat(1 / numParticles, 1, numParticles);
        end
        
        function resampleByESS(obj)
            normalizedESS = obj.getNormalizedESS();
            
            % Only resample if necessary according to the (normalized) ESS
            if normalizedESS < obj.minAllowedNormalizedESS
                obj.resample();
            end
        end
        
        function normalizedESS = getNormalizedESS(obj)
            % Compute the normalized ESS of the particles.
            %
            % Returns:
            %   << normalizedESS (Scalar)
            %      The normalized ESS of the particles.
            
            normalizedESS = 1 / (obj.numParticles * sum(obj.weights.^2, 2));
        end
    end
    
    properties (Access = 'protected')
        % Particles itself.
        particles;
        
        % Associated particle weights.
        weights;
        
        % Number of employed particles for state estimation.
        numParticles;
    end
    
    properties (Access = 'private')
        % Minimum allowed normalized effective sample size.
        minAllowedNormalizedESS;
    end
end
