
classdef GPF < BasePF & GaussianFilter
    % The Gaussian Particle Filter (GPF).
    %
    % GPF Methods:
    %   GPF                       - Class constructor.
    %   getName                   - Get the filter name / description.
    %   setColor                  - Set the filter color / plotting properties.
    %   getColor                  - Get the current filter color / plotting properties.
    %   setState                  - Set the system state.
    %   getState                  - Get the current system state.
    %   getStateDim               - Get the dimension of the current system state.
    %   predict                   - Perform a time update (prediction step).
    %   update                    - Perform a measurement update (filter step) using the given measurement(s).
    %   step                      - Perform a combined time and measurement update.
    %   getPointEstimate          - Get a point estimate of the current system state.
    %   setUseAnalyticSystemModel - Enable or disable the use of analytic moment calculation during a prediction.
    %   getUseAnalyticSystemModel - Get the current use of analytic moment calculation during a prediction.
    %   setStateDecompDim         - Set the dimension of the unobservable part of the system state.
    %   getStateDecompDim         - Get the dimension of the unobservable part of the system state.
    %   setNumParticles           - Set the number of particles used by the filter.
    %   getNumParticles           - Get the current number of particles used by the filter.
    
    % Literature:
    %   Jayesh H. Kotecha and Petar M. Djuric,
    %   Gaussian Particle Filtering,
    %   IEEE Transactions on Signal Processing, Vol. 51, No. 10, Oct 2003, pp. 2592-2601.
    
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
            obj = obj@BasePF(name);
            obj = obj@GaussianFilter(name);
            
            obj.setNumParticles(1000);
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
            % Get the current number of particles used by the filter.
            %
            % Returns:
            %   << numParticles (Positive scalar)
            %      The currentl number of particles used by the filter.
            
            numParticles = obj.numParticles;
        end
    end
    
    methods (Access = 'protected')
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsArbitraryNoise(obj, sysModel)
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
                  predictedStateCov] = predictedMomentsAdditiveNoise(obj, sysModel)
            % Get additive noise moments
            [noiseMean, noiseCov] = sysModel.noise.getMeanAndCovariance();
            
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
        
        function [predictedStateMean, ...
                  predictedStateCov] = predictedMomentsMixedNoise(obj, sysModel)
            % Get additive noise moments
            [addNoiseMean, addNoiseCov] = sysModel.additiveNoise.getMeanAndCovariance();
            
            dimAddNoise = size(addNoiseMean, 1);
            
            obj.checkAdditiveSysNoise(dimAddNoise);
            
            % Sample system noise
            noise = sysModel.noise.drawRndSamples(obj.numParticles);
            
            % Sample system state
            particles = obj.getStateParticles();
            
            % Propagate state particles through system equation
            predictedParticles = sysModel.systemEquation(particles, noise);
            
            % Check predicted state particles
            obj.checkPredictedStateSamples(predictedParticles, obj.numParticles);
            
            [mean, cov] = Utils.getMeanAndCov(predictedParticles);
            
            % Compute predicted state mean
            predictedStateMean = mean + addNoiseMean;
            
            % Compute predicted state covariance
            predictedStateCov = cov + addNoiseCov;
        end
        
        function [updatedMean, ...
                  updatedCov] = performUpdateObservable(obj, measModel, measurements, ...
                                                        priorMean, ~, priorCovSqrt)
            if Checks.isClass(measModel, 'Likelihood')
                % Generate random samples
                particles = Utils.drawGaussianRndSamples(priorMean, ...
                                                         priorCovSqrt, ...
                                                         obj.numParticles);
                
                [updatedMean, ...
                 updatedCov] = obj.updateLikelihood(measModel, measurements, particles);
            else
                obj.errorMeasModel('Likelihood');
            end
        end
        
        function [updatedMean, ...
                  updatedCov] = updateLikelihood(obj, measModel, measurements, particles)
            % Evaluate likelihood
            values = obj.evaluateLikelihood(measModel, measurements, particles, obj.numParticles);
            
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
