
classdef SysModelExample < SystemModel
    methods
        function obj = SysModelExample()
            % Specify the Gaussian system noise.
            % Of course, you do not have to call setNoise() in the constructor.
            % You can also set/change the noise after creating a SysModelExample object.
            obj.setNoise(Gaussian(0, 0.1));
        end
        
        function predictedStates = systemEquation(obj, stateSamples, noiseSamples)
            p = stateSamples(1, :);
            q = stateSamples(2, :);
            w = noiseSamples;
            
            predictedStates = [p .* q.^2 .* w.^2
                               p.^2 + 3*q .* w.^3];
        end
        
        function [stateJacobian, noiseJacobian, ...
                  stateHessians, noiseHessians] = derivative(obj, nominalState, nominalNoise)
            p = nominalState(1);
            q = nominalState(2);
            w = nominalNoise;
            
            % Jacobians
            stateJacobian = [q^2*w^2 2*p*q*w^2
                               2*p     3*w^3  ];
            
            noiseJacobian = [2*p*q^2*w
                              9*q*w^2 ];
            
            % Hessians
            if nargout >= 3
                % Hessian for p_k
                stateHessians(:, :, 1) = [   0    2*q*w^2
                                          2*q*w^2 2*p*w^2];
                
                % Hessian for q_k
                stateHessians(:, :, 2) = [2 0
                                          0 0];
                
                % Hessian for p_k
                noiseHessians(:, :, 1) = 2*p*q^2;
                
                % Hessian for q_k
                noiseHessians(:, :, 2) = 18*q*w;
            end
        end
    end
end
