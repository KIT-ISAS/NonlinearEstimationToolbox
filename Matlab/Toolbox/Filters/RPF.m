
classdef RPF < SIRPF
    % The regularized particle filter (RPF).
    %
    % This implementation uses Gaussian kernels for resampling.
    %
    % RPF Methods:
    %   RPF                        - Class constructor.
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
        function obj = RPF(name)
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
            %      Default name: 'RPF'.
            %
            % Returns:
            %   << obj (RPF)
            %      A new RPF instance.
            
            if nargin < 1
                name = 'RPF';
            end
            
            % Superclass constructor
            obj = obj@SIRPF(name);
        end
    end
    
    methods (Access = 'protected')
        function resample(obj, numParticles)
            % Perform a resampling.
            %
            % Parameters:
            %   >> numParticles (Positive scalar)
            %      The new number of particles.
            %      Default value: the currently selected number of particles.
            
            % Implements the resampling of the regularized PF described in:
            %   Branko Ristic, Sanjeev Arulampalam, and Neil Gordon,
            %   Beyond the Kalman Filter: Particle filters for Tracking Applications,
            %   Artech House Publishers, 2004,
            %   Section 3.5.3
            
            if nargin < 2
                % Amount of particles remains the same
                numParticles = obj.numParticles;
            end
            
            % Compute sample covariance matrix of reweighted particles
            [~, cov] = Utils.getMeanAndCov(obj.particles, obj.weights);
            
            % Assumption: Particle weights are always normalized
            cumWeights = cumsum(obj.weights);
            
            % First, standard SIR-PF resampling is performed
            obj.particles = Utils.systematicResampling(obj.particles, cumWeights, numParticles);
            
            % Second, additional resampling part of the regularized PF
            [covSqrt, isNonPos] = chol(cov, 'Lower');
            
            if ~isNonPos
                % We can only use the resampling of the regularized PF if
                % we have a valid sample covariance matrix. If not, we
                % fall back to the standard SIR-PF resampling.
                
                % Optimal kernel bandwidth (note that we use a Gaussian kernel for the regularization)
                hOpt = (4 / (obj.numParticles * (obj.dimState + 2)))^(1 / (obj.dimState + 4));
                
                obj.particles = obj.particles + (hOpt * covSqrt) * randn(obj.dimState, numParticles);
            end
            
            % Set new number of particles
            obj.numParticles = numParticles;
            
            % Equally weighted particles
            obj.weights = repmat(1 / numParticles, 1, numParticles);
        end
    end
end
