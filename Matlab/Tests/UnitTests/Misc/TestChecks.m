        
classdef TestChecks < matlab.unittest.TestCase
    % Provides unit tests for the Checks class.
    
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
    
    methods (Test)
        function testIsFlag(obj)
            func = @Checks.isFlag;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyTrue(func(obj.flag));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.mat));
        end  
        
        
        function testIsScalar(obj)
            func = @Checks.isScalar;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyTrue(func(obj.scalar));
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.posScalar));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.mat));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsNonNegativeScalar(obj)
            func = @Checks.isNonNegativeScalar;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.posScalar));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.mat));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsPosScalar(obj)
            func = @Checks.isPosScalar;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.posScalar));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.mat));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsScalarIn(obj)
            func = @Checks.isScalarIn;
            
            obj.verifyFalse(func(obj.str, 0, 2));
            
            obj.verifyTrue(func(obj.scalar, -2, 2));
            obj.verifyTrue(func(obj.scalar, -1, 2));
            obj.verifyTrue(func(obj.scalar, -3, -1));
            obj.verifyFalse(func(obj.scalar, 0, 1));
            obj.verifyFalse(func(obj.scalar, -2, -1.5));
            
            obj.verifyFalse(func(obj.flag, 1, 2));
            obj.verifyFalse(func(obj.rowVec, 1, 2));
            obj.verifyFalse(func(obj.colVec, 1, 2));
            obj.verifyFalse(func(obj.mat, 1, 2));
            obj.verifyFalse(func(obj.emptyMat, 1, 2));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        
        function testIsVec(obj)
            func = @Checks.isVec;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyTrue(func(obj.scalar));
            obj.verifyTrue(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 3));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyTrue(func(obj.rowVec));
            obj.verifyTrue(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 2));
            
            obj.verifyTrue(func(obj.nonNegRowVec));
            obj.verifyTrue(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 4));
            
            obj.verifyTrue(func(obj.posRowVec));
            obj.verifyTrue(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyTrue(func(obj.colVec));
            obj.verifyTrue(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 2));
            
            obj.verifyTrue(func(obj.nonNegColVec));
            obj.verifyTrue(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 2));
            
            obj.verifyTrue(func(obj.posColVec));
            obj.verifyTrue(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2)); 
            
            obj.verifyFalse(func(obj.cov));
            obj.verifyFalse(func(obj.cov, 2));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsNonNegativeVec(obj)
            func = @Checks.isNonNegativeVec;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 3));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 2));
            
            obj.verifyTrue(func(obj.nonNegRowVec));
            obj.verifyTrue(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 4));
            
            obj.verifyTrue(func(obj.posRowVec));
            obj.verifyTrue(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 2));
            
            obj.verifyTrue(func(obj.nonNegColVec));
            obj.verifyTrue(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 2));
            
            obj.verifyTrue(func(obj.posColVec));
            obj.verifyTrue(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2)); 
            
            obj.verifyFalse(func(obj.cov));
            obj.verifyFalse(func(obj.cov, 2));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsPosVec(obj)
            func = @Checks.isPosVec;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyFalse(func(obj.nonNegScalar));
            obj.verifyFalse(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 3));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 2));
            
            obj.verifyFalse(func(obj.nonNegRowVec));
            obj.verifyFalse(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 4));
            
            obj.verifyTrue(func(obj.posRowVec));
            obj.verifyTrue(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 2));
            
            obj.verifyFalse(func(obj.nonNegColVec));
            obj.verifyFalse(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 2));
            
            obj.verifyTrue(func(obj.posColVec));
            obj.verifyTrue(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2)); 
            
            obj.verifyFalse(func(obj.cov));
            obj.verifyFalse(func(obj.cov, 2));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end

        
        function testIsColVec(obj)
            func = @Checks.isColVec;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyTrue(func(obj.scalar));
            obj.verifyTrue(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 3));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 2));
            
            obj.verifyFalse(func(obj.nonNegRowVec));
            obj.verifyFalse(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 4));
            
            obj.verifyFalse(func(obj.posRowVec));
            obj.verifyFalse(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyTrue(func(obj.colVec));
            obj.verifyTrue(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 2));
            
            obj.verifyTrue(func(obj.nonNegColVec));
            obj.verifyTrue(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 2));
            
            obj.verifyTrue(func(obj.posColVec));
            obj.verifyTrue(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2)); 
            
            obj.verifyFalse(func(obj.cov));
            obj.verifyFalse(func(obj.cov, 2));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsNonNegativeColVec(obj)
            func = @Checks.isNonNegativeColVec;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 3));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 2));
            
            obj.verifyFalse(func(obj.nonNegRowVec));
            obj.verifyFalse(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 4));
            
            obj.verifyFalse(func(obj.posRowVec));
            obj.verifyFalse(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 2));
            
            obj.verifyTrue(func(obj.nonNegColVec));
            obj.verifyTrue(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 2));
            
            obj.verifyTrue(func(obj.posColVec));
            obj.verifyTrue(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2)); 
            
            obj.verifyFalse(func(obj.cov));
            obj.verifyFalse(func(obj.cov, 2));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsPosColVec(obj)
            func = @Checks.isPosColVec;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyFalse(func(obj.nonNegScalar));
            obj.verifyFalse(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 3));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 2));
            
            obj.verifyFalse(func(obj.nonNegRowVec));
            obj.verifyFalse(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 4));
            
            obj.verifyFalse(func(obj.posRowVec));
            obj.verifyFalse(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 2));
            
            obj.verifyFalse(func(obj.nonNegColVec));
            obj.verifyFalse(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 2));
            
            obj.verifyTrue(func(obj.posColVec));
            obj.verifyTrue(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2)); 
            
            obj.verifyFalse(func(obj.cov));
            obj.verifyFalse(func(obj.cov, 2));
        end
        
        
        function testIsRowVec(obj)
            func = @Checks.isRowVec;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyTrue(func(obj.scalar));
            obj.verifyTrue(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 3));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyTrue(func(obj.rowVec));
            obj.verifyTrue(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 2));
            
            obj.verifyTrue(func(obj.nonNegRowVec));
            obj.verifyTrue(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 4));
            
            obj.verifyTrue(func(obj.posRowVec));
            obj.verifyTrue(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 2));
            
            obj.verifyFalse(func(obj.nonNegColVec));
            obj.verifyFalse(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 2));
            
            obj.verifyFalse(func(obj.posColVec));
            obj.verifyFalse(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2)); 
            
            obj.verifyFalse(func(obj.cov));
            obj.verifyFalse(func(obj.cov, 2));
        end
        
        function testIsNonNegativeRowVec(obj)
            func = @Checks.isNonNegativeRowVec;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 3));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 2));
            
            obj.verifyTrue(func(obj.nonNegRowVec));
            obj.verifyTrue(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 4));
            
            obj.verifyTrue(func(obj.posRowVec));
            obj.verifyTrue(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 2));
            
            obj.verifyFalse(func(obj.nonNegColVec));
            obj.verifyFalse(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 2));
            
            obj.verifyFalse(func(obj.posColVec));
            obj.verifyFalse(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2)); 
            
            obj.verifyFalse(func(obj.cov));
            obj.verifyFalse(func(obj.cov, 2));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsPosRowVec(obj)
            func = @Checks.isPosRowVec;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyFalse(func(obj.nonNegScalar));
            obj.verifyFalse(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 3));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 2));
            
            obj.verifyFalse(func(obj.nonNegRowVec));
            obj.verifyFalse(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 4));
            
            obj.verifyTrue(func(obj.posRowVec));
            obj.verifyTrue(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 2));
            
            obj.verifyFalse(func(obj.nonNegColVec));
            obj.verifyFalse(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 2));
            
            obj.verifyFalse(func(obj.posColVec));
            obj.verifyFalse(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2)); 
            
            obj.verifyFalse(func(obj.cov));
            obj.verifyFalse(func(obj.cov, 2));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        
        function testIsMat(obj)
            func = @Checks.isMat;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1, 1));
            
            obj.verifyTrue(func(obj.scalar));
            obj.verifyTrue(func(obj.scalar, 1, 1));
            obj.verifyFalse(func(obj.scalar, 0, 1));
            
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.nonNegScalar, 1, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 2, 1));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1, 1));
            obj.verifyFalse(func(obj.posScalar, 2, 3));
            
            obj.verifyTrue(func(obj.rowVec));
            obj.verifyTrue(func(obj.rowVec, 1, 3));
            obj.verifyFalse(func(obj.rowVec, 1, 4));
            
            obj.verifyTrue(func(obj.nonNegRowVec));
            obj.verifyTrue(func(obj.nonNegRowVec, 1, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 1, 2));
            
            obj.verifyTrue(func(obj.posRowVec));
            obj.verifyTrue(func(obj.posRowVec, 1, 3));
            obj.verifyFalse(func(obj.posRowVec, 1, 5));
            
            obj.verifyTrue(func(obj.colVec));
            obj.verifyTrue(func(obj.colVec, 3, 1));
            obj.verifyFalse(func(obj.colVec, 3, 0));
            
            obj.verifyTrue(func(obj.nonNegColVec));
            obj.verifyTrue(func(obj.nonNegColVec, 3, 1));
            obj.verifyFalse(func(obj.nonNegColVec, 3, 4));
            
            obj.verifyTrue(func(obj.posColVec));
            obj.verifyTrue(func(obj.posColVec, 3, 1));
            obj.verifyFalse(func(obj.posColVec, 3, 2));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2, 3));
            
            obj.verifyTrue(func(obj.mat));
            obj.verifyTrue(func(obj.mat, 2, 3));
            obj.verifyFalse(func(obj.mat, 3, 2));
            
            obj.verifyTrue(func(obj.squareMat));
            obj.verifyTrue(func(obj.squareMat, 2, 2));
            obj.verifyFalse(func(obj.squareMat, 1, 2));
            
            obj.verifyTrue(func(obj.cov));
            obj.verifyTrue(func(obj.cov, 2, 2));
            obj.verifyFalse(func(obj.cov, 2, 4));
        end
        
        function testIsFixedRowMat(obj)
            func = @Checks.isFixedRowMat;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyTrue(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyTrue(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 2));
            
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyTrue(func(obj.rowVec, 1));
            obj.verifyFalse(func(obj.rowVec, 4));
            
            obj.verifyTrue(func(obj.nonNegRowVec, 1));
            obj.verifyFalse(func(obj.nonNegRowVec, 2));
            
            obj.verifyTrue(func(obj.posRowVec, 1));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyTrue(func(obj.colVec, 3));
            obj.verifyFalse(func(obj.colVec, 0));
            
            obj.verifyTrue(func(obj.nonNegColVec, 3));
            obj.verifyFalse(func(obj.nonNegColVec, 4));
            
            obj.verifyTrue(func(obj.posColVec, 3));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyTrue(func(obj.mat, 2));
            obj.verifyFalse(func(obj.mat, 3));
            
            obj.verifyTrue(func(obj.squareMat, 2));
            obj.verifyFalse(func(obj.squareMat, 4));
            
            obj.verifyTrue(func(obj.cov, 2));
            obj.verifyFalse(func(obj.cov, 0));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsFixedColMat(obj)
            func = @Checks.isFixedColMat;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyTrue(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyTrue(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 2));
            
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyTrue(func(obj.rowVec, 3));
            obj.verifyFalse(func(obj.rowVec, 4));
            
            obj.verifyTrue(func(obj.nonNegRowVec, 3));
            obj.verifyFalse(func(obj.nonNegRowVec, 2));
            
            obj.verifyTrue(func(obj.posRowVec, 3));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyTrue(func(obj.colVec, 1));
            obj.verifyFalse(func(obj.colVec, 0));
            
            obj.verifyTrue(func(obj.nonNegColVec, 1));
            obj.verifyFalse(func(obj.nonNegColVec, 4));
            
            obj.verifyTrue(func(obj.posColVec, 1));
            obj.verifyFalse(func(obj.posColVec, 2));
            
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyTrue(func(obj.mat, 3));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyTrue(func(obj.squareMat, 2));
            obj.verifyFalse(func(obj.squareMat, 5));
            
            obj.verifyTrue(func(obj.cov, 2));
            obj.verifyFalse(func(obj.cov, 1));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsSquareMat(obj)
            func = @Checks.isSquareMat;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyTrue(func(obj.scalar));
            obj.verifyTrue(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyTrue(func(obj.nonNegScalar));
            obj.verifyTrue(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 2));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 1));
            
            obj.verifyFalse(func(obj.nonNegRowVec));
            obj.verifyFalse(func(obj.nonNegRowVec, 2));
            
            obj.verifyFalse(func(obj.posRowVec));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 1));
            
            obj.verifyFalse(func(obj.nonNegColVec));
            obj.verifyFalse(func(obj.nonNegColVec, 3));
            
            obj.verifyFalse(func(obj.posColVec));
            obj.verifyFalse(func(obj.posColVec, 1));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyTrue(func(obj.squareMat));
            obj.verifyTrue(func(obj.squareMat, 2));
            obj.verifyFalse(func(obj.squareMat, 1));
            
            obj.verifyTrue(func(obj.cov));
            obj.verifyTrue(func(obj.cov, 2));
            obj.verifyFalse(func(obj.cov, 3));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        
        function testIsCov(obj)
            func = @Checks.isCov;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyFalse(func(obj.nonNegScalar));
            obj.verifyFalse(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 2));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 1));
            
            obj.verifyFalse(func(obj.nonNegRowVec));
            obj.verifyFalse(func(obj.nonNegRowVec, 2));
            
            obj.verifyFalse(func(obj.posRowVec));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 1));
            
            obj.verifyFalse(func(obj.nonNegColVec));
            obj.verifyFalse(func(obj.nonNegColVec, 3));
            
            obj.verifyFalse(func(obj.posColVec));
            obj.verifyFalse(func(obj.posColVec, 1));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2));
            obj.verifyFalse(func(obj.squareMat, 1));
            
            obj.verifyTrue(func(obj.cov));
            obj.verifyTrue(func(obj.cov, 2));
            obj.verifyFalse(func(obj.cov, 3));
            
            obj.verifyFalse(func(obj.cov3D));
            
            obj.verifyFalse(func(obj.nonCov3D));
        end
        
        function testIsCov3D(obj)
            func = @Checks.isCov3D;
            
            obj.verifyFalse(func(obj.str));
            
            obj.verifyFalse(func(obj.flag));
            obj.verifyFalse(func(obj.flag, 1));
            
            obj.verifyFalse(func(obj.scalar));
            obj.verifyFalse(func(obj.scalar, 1));
            obj.verifyFalse(func(obj.scalar, 0));
            
            obj.verifyFalse(func(obj.nonNegScalar));
            obj.verifyFalse(func(obj.nonNegScalar, 1));
            obj.verifyFalse(func(obj.nonNegScalar, 2));
            
            obj.verifyTrue(func(obj.posScalar));
            obj.verifyTrue(func(obj.posScalar, 1));
            obj.verifyFalse(func(obj.posScalar, 2));
            
            obj.verifyFalse(func(obj.rowVec));
            obj.verifyFalse(func(obj.rowVec, 1));
            
            obj.verifyFalse(func(obj.nonNegRowVec));
            obj.verifyFalse(func(obj.nonNegRowVec, 2));
            
            obj.verifyFalse(func(obj.posRowVec));
            obj.verifyFalse(func(obj.posRowVec, 5));
            
            obj.verifyFalse(func(obj.colVec));
            obj.verifyFalse(func(obj.colVec, 1));
            
            obj.verifyFalse(func(obj.nonNegColVec));
            obj.verifyFalse(func(obj.nonNegColVec, 3));
            
            obj.verifyFalse(func(obj.posColVec));
            obj.verifyFalse(func(obj.posColVec, 1));
            
            obj.verifyFalse(func(obj.emptyMat));
            obj.verifyFalse(func(obj.emptyMat, 2));
            
            obj.verifyFalse(func(obj.mat));
            obj.verifyFalse(func(obj.mat, 2));
            
            obj.verifyFalse(func(obj.squareMat));
            obj.verifyFalse(func(obj.squareMat, 2));
            obj.verifyFalse(func(obj.squareMat, 1));
            
            obj.verifyTrue(func(obj.cov));
            obj.verifyTrue(func(obj.cov, 2));
            obj.verifyTrue(func(obj.cov, 2, 1));
            obj.verifyFalse(func(obj.cov, 3));
            obj.verifyFalse(func(obj.cov, 2, 2));
            
            obj.verifyTrue(func(obj.cov3D));
            obj.verifyTrue(func(obj.cov3D, 2));
            obj.verifyTrue(func(obj.cov3D, 2, 3));
            obj.verifyFalse(func(obj.cov3D, 3));
            obj.verifyFalse(func(obj.cov3D, 2, 4));
            
            obj.verifyFalse(func(obj.nonCov3D));
            obj.verifyFalse(func(obj.nonCov3D, 2));
            obj.verifyFalse(func(obj.nonCov3D, 3));
            obj.verifyFalse(func(obj.nonCov3D, 2, 1));
            obj.verifyFalse(func(obj.nonCov3D, 2, 2));
        end
        
        
        function testIsClass(obj)
            func = @Checks.isClass;
            
            obj.verifyTrue(func(obj.str, 'char', 6));
            
            obj.verifyTrue(func(obj.scalar, 'double'));
            obj.verifyFalse(func(obj.scalar, 'single'));
            obj.verifyTrue(func(obj.scalar, 'double', 1));
            obj.verifyFalse(func(obj.scalar, 'double', 4));
            
            obj.verifyFalse(func(obj.mat, 'double'));
            obj.verifyTrue(func(obj.mat, 'double', 6));
            obj.verifyFalse(func(obj.mat, 'double', 3));
        end
    end
      
    properties (Constant)
        str          = 'string';
        flag         = true;
        scalar       = -1;
        nonNegScalar = 0;
        posScalar    = 2;
        rowVec       = [1.4 2.6 -5];
        nonNegRowVec = [0 1 3];
        posRowVec    = [1 2 3];
        colVec       = [-3 9.2 71.4]';
        nonNegColVec = [0 1 3]';
        posColVec    = [5 6 7]';
        emptyMat     = [];
        mat          = [1 2 3; 4 5 6];
        squareMat    = [1 2; 3 4];
        cov          = [2 0.5; 0.5 1.2];
        cov3D        = cat(3, [2 0.5; 0.5 1.2], eye(2), 3 * eye(2));
        nonCov3D     = cat(3, [2 0.5; 0.5 1.2], zeros(2, 2));
    end
end
