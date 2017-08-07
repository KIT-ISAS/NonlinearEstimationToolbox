
classdef GPF < GaussianFilter & ParticleFilter
    % The Gaussian particle filter (GPF).
    %
    % GPF Methods:
    %   GPF                         - Class constructor.
    %   copy                        - Copy a Filter instance.
    %   copyWithName                - Copy a Filter instance and give the copy a new name/description.
    %   getName                     - Get the filter name/description.
    %   setColor                    - Set the filter color/plotting properties.
    %   getColor                    - Get the filter color/plotting properties.
    %   setState                    - Set the system state.
    %   getState                    - Get the system state.
    %   getStateDim                 - Get the dimension of the system state.
    %   getStateMeanAndCov          - Get mean and covariance matrix of the system state.
    %   predict                     - Perform a state prediction.
    %   update                      - Perform a measurement update.
    %   step                        - Perform a combined state prediction and measurement update.
    %   setStateDecompDim           - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim           - Get the dimension of the unobservable part of the system state.
    %   setPredictionPostProcessing - Set a post-processing method for the state prediction.
    %   getPredictionPostProcessing - Get the post-processing method for the state prediction.
    %   setUpdatePostProcessing     - Set a post-processing method for the measurement update.
    %   getUpdatePostProcessing     - Get the post-processing method for the measurement update.
    %   setNumParticles             - Set the number of particles used by the filter.
    %   getNumParticles             - Get the number of particles used by the filter.
    
    % Literature:
    %   Jayesh H. Kotecha and Petar M. Djuric,
    %   Gaussian Particle Filtering,
    %   IEEE Transactions on Signal Processing, vol. 51, no. 10, pp. 2592-2601, Oct. 2003.
    
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
        function obj = GPF(name)
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
            %      Default name: 'GPF'.
            %
            % Returns:
            %   << obj (GPF)
            %      A new GPF instance.
            
            if nargin < 1
                name = 'GPF';
            end
            
            % Call superclass constructors
            obj = obj@GaussianFilter(name);
            obj = obj@ParticleFilter(name);
            
            obj.numParticles = 1000;
        end
        
        function setNumParticles(obj, numParticles)
            % Set the number of particles used by the filter.
            %
            % By default, 1000 particles are used.
            %
            % Parameters:
            %   >> numParticles (Positive scalar)
            %      The new number of particles used by the filter.
            
            if ~Checks.isPosScalar(numParticles)
                obj.error('InvalidNumberOfParticles', ...
                          'numParticles must be a positive scalar.');
            end
            
            obj.numParticles = ceil(numParticles);
        end
        
        function numParticles = getNumParticles(obj)
            % Get the number of particles used by the filter.
            %
            % Returns:
            %   << numParticles (Positive scalar)
            %      The number of particles used by the filter.
            
            numParticles = obj.numParticles;
        end
    end
    
    methods (Access = 'protected')
        function [predictedStateMean, ...
                  predictedStateCov] = predictSysModel(obj, sysModel)
            % Sample system noise
            noise = sysModel.noise.drawRndSamples(obj.numParticles);
            
            % Sample system state
            particles = obj.getStateParticles();
            
            % Propagate state particles through system equation
            predictedParticles = sysModel.systemEquation(particles, noise);
            
            % Check predicted state particles
            obj.checkPredictedStateSamples(predictedParticles, obj.numParticles);
            
            % Compute predicted state mean and covariance
            [predictedStateMean, ...
             predictedStateCov] = Utils.getMeanAndCov(predictedParticles);
        end
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictAddNoiseSysModel(obj, sysModel)
            % Get additive noise moments
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCov();
            
            dimNoise = size(noiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimNoise);
            
            % Sample system state
            particles = obj.getStateParticles();
            
            % Propagate state particles through deterministic system equation
            predictedParticles = sysModel.systemEquation(particles);
            
            % Check predicted state particles
            obj.checkPredictedStateSamples(predictedParticles, obj.numParticles);
            
            [mean, cov] = Utils.getMeanAndCov(predictedParticles);
            
            % Compute predicted state mean
            predictedStateMean = mean + noiseMean;
            
            % Compute predicted state covariance
            predictedStateCov = cov + noiseCov;
        end
        
        function [updatedMean, ...
                  updatedCov] = performUpdateObservable(obj, measModel, measurement, ...
                                                        priorMean, ~, priorCovSqrt)
            if Checks.isClass(measModel, 'Likelihood')
                % Generate random samples
                particles = Utils.drawGaussianRndSamples(priorMean, ...
                                                         priorCovSqrt, ...
                                                         obj.numParticles);
                
                [updatedMean, ...
                 updatedCov] = obj.updateLikelihood(measModel, measurement, particles);
            else
                obj.errorMeasModel('Likelihood');
            end
        end
        
        function [updatedMean, ...
                  updatedCov] = updateLikelihood(obj, measModel, measurement, particles)
            % Evaluate likelihood
            values = obj.evaluateLikelihood(measModel, measurement, particles, obj.numParticles);
            
            % Normalize particle weights
            sumWeights = sum(values);
            
            if sumWeights <= 0
                obj.ignoreMeas('Sum of computed posterior particle weights is not positive.');
            end
            
            weights = values / sumWeights;
            
            % Compute updated state mean and covariance
            [updatedMean, ...
             updatedCov] = Utils.getMeanAndCov(particles, weights);
        end
        
        function particles = getStateParticles(obj)
            % Generate Gaussian random samples
            particles = Utils.drawGaussianRndSamples(obj.stateMean, ...
                                                     obj.stateCovSqrt, ...
                                                     obj.numParticles);
        end
    end
    
    properties (SetAccess = 'private', GetAccess = 'protected')
        % Number of particles to use during prediction and update.
        numParticles;
    end
end
