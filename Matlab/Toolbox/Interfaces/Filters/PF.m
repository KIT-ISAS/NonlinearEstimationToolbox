
classdef PF < BasePF
    % Abstract base class for Particle Filters where the state estimate is a set of weighted particles.
    %
    % PF Methods:
    %   PF                         - Class constructor.
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
        function obj = PF(name)
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
            %   << obj (PF)
            %      A new PF instance.
            
            % Call superclass constructor
            obj = obj@BasePF(name);
            
            obj.dimState  = 0;
            obj.particles = [];
            obj.weights   = [];
            
            obj.setNumParticles(1000);
            
            obj.setMinAllowedNormalizedESS(0.5);
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
            % Get the current number of particles used by the filter.
            %
            % Returns:
            %   << numParticles (Positive scalar)
            %      The currentl number of particles used by the filter.
            
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
            % Get the current minimum allowed normalized effective sample size (ESS).
            %
            % Returns:
            %   << minAllowedNormalizedESS (Scalar)
            %      The current minimum allowed noramlized ESS between 0 and 1.
            
            minAllowedNormalizedESS = obj.minAllowedNormalizedESS;
        end
        
        function setState(obj, state)
            if ~Checks.isClass(state, 'Distribution');
                obj.error('UnsupportedSystemState', ...
                          'state must be a subclass of Distribution.');
            end
            
            if Checks.isClass(state, 'DiracMixture')
                % Directly use the Dirac mixture components as system state
                [obj.particles, obj.weights] = state.getComponents();
            else
                % Draw random particles
                obj.particles = state.drawRndSamples(obj.numParticles);
                
                % Particles are equally weighted
                obj.weights = repmat(1 / obj.numParticles, 1, obj.numParticles);
            end
            
            obj.dimState = state.getDimension();
        end
        
        function state = getState(obj)
            state = DiracMixture(obj.particles, obj.weights);
        end
        
        function [pointEstimate, uncertainty] = getPointEstimate(obj)
            % Get a point estimate of the current system state.
            %
            % Returns:
            %   << pointEstimate (Column vector)
            %      Mean of the current particle set.
            %
            %   << uncertainty (Positive definite matrix)
            %      Covariance of the current particle set.
            
            if nargout == 1
                pointEstimate = Utils.getMeanAndCov(obj.particles, obj.weights);
            else
                [pointEstimate, uncertainty] = Utils.getMeanAndCov(obj.particles, obj.weights);
            end
        end
    end
    
    methods (Access = 'protected')    
        function normalizedESS = getNormalizedESS(obj)
            % Compute the normalized ESS of the current particle set.
            %
            % Returns:
            %   << normalizedESS (Scalar)
            %      The normalized ESS of the current particle set.
            
            normalizedESS = 1 / (obj.numParticles * sum(obj.weights.^2, 2));
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
    end
    
    properties (Access = 'protected')
        % Particles itself.
        particles;
        
        % Associated particle weights.
        weights;
        
        % Number of employed particles for state estimation.
        numParticles;
    end
    
    properties (SetAccess = 'private', GetAccess = 'protected')
        % Minimum allowed normalized effective sample size.
        minAllowedNormalizedESS;
    end
end
