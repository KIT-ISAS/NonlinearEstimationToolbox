
classdef TestRURUF < TestRecursiveUpdateFilter
    % Provides unit tests for the RURUF class.
    
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
    
    methods (Test)
        function testSetNumSamplesFactors(obj)
            f = obj.initFilter();
            
            f.setNumSamplesFactors(10);
            
            [factorPrediction, ...
             factorUpdate] = f.getNumSamplesFactors();
            
            obj.verifyEqual(factorPrediction, 10);
            obj.verifyEqual(factorUpdate, 10);
        end
        
        function testSetNumSamplesFactorsBoth(obj)
            f = obj.initFilter();
            
            f.setNumSamplesFactors(10, 6);
            
            [factorPrediction, ...
             factorUpdate] = f.getNumSamplesFactors();
            
            obj.verifyEqual(factorPrediction, 10);
            obj.verifyEqual(factorUpdate, 6);
        end
    end
    
    methods (Access = 'protected')
        function f = initFilter(~)
            f = RURUF();
        end
        
        function defaultConstructorTests(obj, f)
            % Call superclass tests
            obj.defaultConstructorTests@TestRecursiveUpdateFilter(f);
            
            % RUKF-related tests
            obj.verifyEqual(f.getName(), 'RURUF');
            
            [factorPrediction, ...
             factorUpdate] = f.getNumSamplesFactors();
            
            obj.verifyEqual(factorPrediction, 5);
            obj.verifyEqual(factorUpdate, 5);
        end
    end
end
