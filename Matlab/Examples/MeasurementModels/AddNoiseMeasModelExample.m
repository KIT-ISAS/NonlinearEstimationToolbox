
classdef AddNoiseMeasModelExample < AdditiveNoiseMeasurementModel
    methods
        function obj = AddNoiseMeasModelExample()
            % Specify the Gaussian measurement noise.
            % Of course, you do not have to call setNoise() in the constructor.
            % You can also set/change the noise after creating a AddNoiseMeasModelExample object.
            obj.setNoise(Gaussian(zeros(2, 1), [0.01, 0.1]));
        end
        
        function measurements = measurementEquation(obj, stateSamples)
            p = stateSamples(1, :);
            q = stateSamples(2, :);
            
            measurements = [p .* q.^2
                            p.^2 + 3*q];
        end
        
        function [stateJacobian, stateHessians] = derivative(obj, nominalState)
            p = nominalState(1);
            q = nominalState(2);
            
            % Jacobian
            stateJacobian = [q^2 2*p*q
                             2*p   3  ];
            
            if nargout == 2
                % Hessian for p_k
                stateHessians(:, :, 1) = [ 0  2*q
                                          2*q 2*p];
                
                % Hessian for q_k
                stateHessians(:, :, 2) = [2 0
                                          0 0];
            end
        end
    end
end
