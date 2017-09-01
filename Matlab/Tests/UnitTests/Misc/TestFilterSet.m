
classdef TestFilterSet < matlab.unittest.TestCase
    % Provides unit tests for the FilterSet class.
    
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
        function testConstructor(obj)
            set = FilterSet();
            
            obj.verifyEqual(set.getNumFilters(), 0);
            obj.verifyEqual(set.getStateDim(), 0);
            obj.verifyEqual(set.getNames(), cell(1, 0));
            obj.verifyEqual(set.getStates(), cell(1, 0));
            
            [stateMeans, stateCovs, stateCovSqrts] = set.getStatesMeanAndCov();
            obj.verifyEmpty(stateMeans);
            obj.verifyEmpty(stateCovs);
            obj.verifyEmpty(stateCovSqrts);
        end
        
        
        function testAdd(obj)
            set = FilterSet();
            f   = EKF('KF');
            
            set.add(f);
            
            obj.verifyEqual(set.getNumFilters(), 1);
            obj.verifyEqual(set.getStateDim(), 0);
            obj.verifyEqual(set.getNames(), { 'KF' });
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            obj.verifyEmpty(stateMeans);
            obj.verifyEmpty(stateCovs);
        end
        
        function testAddMultiple(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = EKF('A');
            
            set.add(f);
            obj.verifyEqual(set.getNumFilters(), 1);
            obj.verifyEqual(set.getStateDim(), 0);
            obj.verifyEqual(set.getNames(), { 'B' });
            
            set.add(f2);
            obj.verifyEqual(set.getNumFilters(), 2);
            obj.verifyEqual(set.getStateDim(), 0);
            obj.verifyEqual(set.getNames(), { 'A', 'B' });
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            obj.verifyEmpty(stateMeans);
            obj.verifyEmpty(stateCovs);
        end
        
        function testAddSameName(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF');
            
            set.add(f);
            
            obj.verifyError(@() set.add(f2), 'FilterSet:InvalidFilterName');
        end
        
        function testAddInvalidFilter(obj)
            set = FilterSet();
            
            obj.verifyError(@() set.add(ones(2)), 'FilterSet:InvalidFilter');
            obj.verifyError(@() set.add(Gaussian()), 'FilterSet:InvalidFilter');
        end
        
        
        function testRemove(obj)
            set = FilterSet();
            f   = EKF('KF');
            
            set.add(f);
            
            set.remove('KF');
            
            obj.verifyEqual(set.getNumFilters(), 0);
            obj.verifyEqual(set.getStateDim(), 0);
            obj.verifyEqual(set.getNames(), cell(1, 0));
            obj.verifyEqual(set.getStates(), cell(1, 0));
        end
        
        function testRemoveMultiple(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            set.remove('A');
            
            obj.verifyEqual(set.getNumFilters(), 1);
            obj.verifyEqual(set.getStateDim(), 0);
            obj.verifyEqual(set.getNames(), { 'B' });
            
            t = set.get(1);
            obj.verifyInstanceOf(t, 'EKF');
            
            set.remove('B');
            
            obj.verifyEqual(set.getNumFilters(), 0);
            obj.verifyEqual(set.getStateDim(), 0);
            obj.verifyEqual(set.getNames(), cell(1, 0));
            obj.verifyEqual(set.getStates(), cell(1, 0));
        end
        
        function testRemoveNoName(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            obj.verifyError(@() set.remove('C'), 'FilterSet:NoFilter');
        end
        
        function testRemoveInvalidName(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            obj.verifyError(@() set.remove(ones(2)), 'FilterSet:InvalidFilterName');
            obj.verifyError(@() set.remove(Gaussian()), 'FilterSet:InvalidFilterName');
        end
        
        
        function testGet(obj)
            set = FilterSet();
            f   = EKF('KF');
            
            set.add(f);
            
            t = set.get('KF');
            obj.verifyInstanceOf(t, 'EKF');
            
            t = set.get(1);
            obj.verifyInstanceOf(t, 'EKF');
        end
        
        function testGetMultiple(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            t = set.get('A');
            obj.verifyInstanceOf(t, 'SIRPF');
            
            t = set.get(1);
            obj.verifyInstanceOf(t, 'SIRPF');
            
            t = set.get('B');
            obj.verifyInstanceOf(t, 'EKF');
            
            t = set.get(2);
            obj.verifyInstanceOf(t, 'EKF');
        end
        
        function testGetOutOfRangeIndex(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            obj.verifyError(@() set.get(3), 'FilterSet:NoFilter');
        end
        
        function testGetNoName(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            obj.verifyError(@() set.get('C'), 'FilterSet:NoFilter');
        end
        
        function testGetIdZero(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            obj.verifyError(@() set.get(0), 'FilterSet:InvalidIdentifier');
        end
        
        function testGetInvalidId(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            obj.verifyError(@() set.get(ones(2)), 'FilterSet:InvalidIdentifier');
            obj.verifyError(@() set.get(Gaussian()), 'FilterSet:InvalidIdentifier');
        end
        
        
        function testGetIndex(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            
            obj.verifyEqual(set.getIndex('B'), 1);
            
            set.add(f2);
            
            obj.verifyEqual(set.getIndex('A'), 1);
            obj.verifyEqual(set.getIndex('B'), 2);
        end
        
        function testGetIndexNoName(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            obj.verifyError(@() set.getIndex('C'), 'FilterSet:NoFilter');
        end
        
        function testGetIndexInvalidName(obj)
            set = FilterSet();
            f   = EKF('B');
            f2  = SIRPF('A');
            
            set.add(f);
            set.add(f2);
            
            obj.verifyError(@() set.getIndex(ones(2)), 'FilterSet:InvalidFilterName');
            obj.verifyError(@() set.getIndex(Gaussian()), 'FilterSet:InvalidFilterName');
        end
        
        
        function testSetStates(obj)
            set = FilterSet();
            f   = EKF('KF');
            
            set.add(f);
            
            mean    = ones(2, 1);
            cov     = 2 * eye(2);
            covSqrt = sqrt(cov);
            
            set.setStates(Gaussian(mean, cov));
            
            obj.verifyEqual(set.getStateDim(), 2);
            
            states = set.getStates();
            
            obj.verifyInstanceOf(states, 'cell');
            obj.verifySize(states, [1 1]);
            
            state = states{1};
            obj.verifyInstanceOf(state, 'Gaussian');
            
            [stateMean, stateCov] = state.getMeanAndCov();
            obj.verifyEqual(stateMean, mean);
            obj.verifyEqual(stateCov, cov);
            
            [stateMeans, stateCovs, stateCovSqrts] = set.getStatesMeanAndCov();
            obj.verifyEqual(stateMeans, mean);
            obj.verifyEqual(stateCovs, cov);
            obj.verifyEqual(stateCovSqrts, covSqrt);
        end
        
        function testSetStatesMultiple(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = SIRPF('SIRPF');
            
            set.add(f);
            set.add(f2);
            
            mean = ones(4, 1);
            cov  = 2 * eye(4);
            
            set.setStates(Gaussian(mean, cov));
            
            obj.verifyEqual(set.getStateDim(), 4);
            
            states = set.getStates();
            
            obj.verifyInstanceOf(states, 'cell');
            obj.verifySize(states, [1 2]);
            
            state = states{1};
            obj.verifyInstanceOf(state, 'Gaussian');
            
            [stateMean1, stateCov1, stateCovSqrt1] = state.getMeanAndCov();
            obj.verifyEqual(stateMean1, mean);
            obj.verifyEqual(stateCov1, cov);
            
            state2 = states{2};
            obj.verifyInstanceOf(state2, 'DiracMixture');
            obj.verifyEqual(state2.getDim(), 4);
            obj.verifyEqual(state2.getNumComponents(), f2.getNumParticles());
            
            [stateMean2, stateCov2, stateCovSqrt2] = state2.getMeanAndCov();
            
            means    = [stateMean1 stateMean2];
            covs     = cat(3, stateCov1, stateCov2);
            covSqrts = cat(3, stateCovSqrt1, stateCovSqrt2);
            
            [stateMeans, stateCovs, stateCovSqrts] = set.getStatesMeanAndCov();
            obj.verifySize(stateMeans, [4 2]);
            obj.verifySize(stateCovs, [4 4 2]);
            obj.verifySize(stateCovSqrts, [4 4 2]);
            
            obj.verifyEqual(stateMeans, means, 'AbsTol', 1e-12);
            obj.verifyEqual(stateCovs, covs, 'AbsTol', 1e-12);
            obj.verifyEqual(stateCovSqrts, covSqrts, 'AbsTol', 1e-12);
        end
        
        
        function testSetStatesMeanAndCov(obj)
            set = FilterSet();
            f   = EKF('KF');
            
            set.add(f);
            
            mean    = ones(2, 1);
            cov     = 2 * eye(2);
            covSqrt = sqrt(cov);
            
            set.setStatesMeanAndCov(mean, cov, covSqrt);
            
            obj.verifyEqual(set.getStateDim(), 2);
            
            states = set.getStates();
            
            obj.verifyInstanceOf(states, 'cell');
            obj.verifySize(states, [1 1]);
            
            state = states{1};
            obj.verifyInstanceOf(state, 'Gaussian');
            
            [stateMean, stateCov] = state.getMeanAndCov();
            obj.verifyEqual(stateMean, mean);
            obj.verifyEqual(stateCov, cov);
            
            [stateMeans, stateCovs, stateCovSqrts] = set.getStatesMeanAndCov();
            obj.verifyEqual(stateMeans, mean);
            obj.verifyEqual(stateCovs, cov);
            obj.verifyEqual(stateCovSqrts, covSqrt);
        end
        
        function testSetStatesMeanAndCovNoCovSqrt(obj)
            set = FilterSet();
            f   = EKF('KF');
            
            set.add(f);
            
            mean    = ones(2, 1);
            cov     = 2 * eye(2);
            covSqrt = sqrt(cov);
            
            set.setStatesMeanAndCov(mean, cov);
            
            obj.verifyEqual(set.getStateDim(), 2);
            
            states = set.getStates();
            
            obj.verifyInstanceOf(states, 'cell');
            obj.verifySize(states, [1 1]);
            
            state = states{1};
            obj.verifyInstanceOf(state, 'Gaussian');
            
            [stateMean, stateCov] = state.getMeanAndCov();
            obj.verifyEqual(stateMean, mean);
            obj.verifyEqual(stateCov, cov);
            
            [stateMeans, stateCovs, stateCovSqrts] = set.getStatesMeanAndCov();
            obj.verifyEqual(stateMeans, mean);
            obj.verifyEqual(stateCovs, cov);
            obj.verifyEqual(stateCovSqrts, covSqrt);
        end
        
        function testSetStatesMeanAndCovMultiple(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = SIRPF('SIRPF');
            
            set.add(f);
            set.add(f2);
            
            mean    = ones(4, 1);
            cov     = 2 * eye(4);
            covSqrt = sqrt(cov);
            
            set.setStatesMeanAndCov(mean, cov, covSqrt);
            
            obj.verifyEqual(set.getStateDim(), 4);
            
            states = set.getStates();
            
            obj.verifyInstanceOf(states, 'cell');
            obj.verifySize(states, [1 2]);
            
            state = states{1};
            obj.verifyInstanceOf(state, 'Gaussian');
            
            [stateMean1, stateCov1, stateCovSqrt1] = state.getMeanAndCov();
            obj.verifyEqual(stateMean1, mean);
            obj.verifyEqual(stateCov1, cov);
            
            state2 = states{2};
            obj.verifyInstanceOf(state2, 'DiracMixture');
            obj.verifyEqual(state2.getDim(), 4);
            obj.verifyEqual(state2.getNumComponents(), f2.getNumParticles());
            
            [stateMean2, stateCov2, stateCovSqrt2] = state2.getMeanAndCov();
            
            means    = [stateMean1 stateMean2];
            covs     = cat(3, stateCov1, stateCov2);
            covSqrts = cat(3, stateCovSqrt1, stateCovSqrt2);
            
            [stateMeans, stateCovs, stateCovSqrts] = set.getStatesMeanAndCov();
            obj.verifySize(stateMeans, [4 2]);
            obj.verifySize(stateCovs, [4 4 2]);
            obj.verifySize(stateCovSqrts, [4 4 2]);
            
            obj.verifyEqual(stateMeans, means, 'AbsTol', 1e-12);
            obj.verifyEqual(stateCovs, covs, 'AbsTol', 1e-12);
            obj.verifyEqual(stateCovSqrts, covSqrts, 'AbsTol', 1e-12);
        end
        
        function testSetStatesMeanAndCovMultipleNoCovSqrt(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = SIRPF('SIRPF');
            
            set.add(f);
            set.add(f2);
            
            mean = ones(4, 1);
            cov  = 2 * eye(4);
            
            set.setStatesMeanAndCov(mean, cov);
            
            obj.verifyEqual(set.getStateDim(), 4);
            
            states = set.getStates();
            
            obj.verifyInstanceOf(states, 'cell');
            obj.verifySize(states, [1 2]);
            
            state = states{1};
            obj.verifyInstanceOf(state, 'Gaussian');
            
            [stateMean1, stateCov1, stateCovSqrt1] = state.getMeanAndCov();
            obj.verifyEqual(stateMean1, mean);
            obj.verifyEqual(stateCov1, cov);
            
            state2 = states{2};
            obj.verifyInstanceOf(state2, 'DiracMixture');
            obj.verifyEqual(state2.getDim(), 4);
            obj.verifyEqual(state2.getNumComponents(), f2.getNumParticles());
            
            [stateMean2, stateCov2, stateCovSqrt2] = state2.getMeanAndCov();
            
            means    = [stateMean1 stateMean2];
            covs     = cat(3, stateCov1, stateCov2);
            covSqrts = cat(3, stateCovSqrt1, stateCovSqrt2);
            
            [stateMeans, stateCovs, stateCovSqrts] = set.getStatesMeanAndCov();
            obj.verifySize(stateMeans, [4 2]);
            obj.verifySize(stateCovs, [4 4 2]);
            obj.verifySize(stateCovSqrts, [4 4 2]);
            
            obj.verifyEqual(stateMeans, means, 'AbsTol', 1e-12);
            obj.verifyEqual(stateCovs, covs, 'AbsTol', 1e-12);
            obj.verifyEqual(stateCovSqrts, covSqrts, 'AbsTol', 1e-12);
        end
        
        
        function testPredict(obj)
            set = FilterSet();
            f   = EKF('KF');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            set.add(f);
            set.setStates(initState);
            runtimes = set.predict(sysModel);
            
            obj.verifySize(runtimes, [1 1]);
            obj.verifyGreaterThan(runtimes, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt = EKF('GT');
            gt.setState(initState);
            gt.predict(sysModel);
            [gtMean, gtCov] = gt.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, gtMean);
            obj.verifyEqual(stateCovs, gtCov);
        end
        
        function testPredictMultiple(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState1 = Gaussian(ones(4, 1), 2 * eye(4));
            initState2 = Gaussian(3 * ones(4, 1), 4 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            set.add(f);
            set.add(f2);
            set.setStates(initState1); % To set joint state dim
            
            f.setState(initState1);
            f2.setState(initState2);
            runtimes = set.predict(sysModel);
            
            obj.verifySize(runtimes, [1 2]);
            obj.verifyGreaterThan(runtimes, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt1 = EKF('GT1');
            gt1.setState(initState1);
            gt1.predict(sysModel);
            [gtMean1, gtCov1] = gt1.getStateMeanAndCov();
            
            gt2 = EKF('GT2');
            gt2.setState(initState2);
            gt2.predict(sysModel);
            [gtMean2, gtCov2] = gt2.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, [gtMean1 gtMean2]);
            obj.verifyEqual(stateCovs, cat(3, gtCov1, gtCov2));
        end
        
        
        function testPredictSingleName(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(2 * ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            runtime = set.predictSingle('KF2', sysModel);
            
            obj.verifySize(runtime, [1 1]);
            obj.verifyGreaterThan(runtime, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt = EKF('GT');
            gt.setState(initState);
            gt.predict(sysModel);
            [gtMean, gtCov] = gt.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, [2 * ones(4, 1) gtMean]);
            obj.verifyEqual(stateCovs, cat(3, 2 * eye(4), gtCov));
        end
        
        function testPredictSingleId(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(2 * ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            runtime = set.predictSingle(2, sysModel);
            
            obj.verifySize(runtime, [1 1]);
            obj.verifyGreaterThan(runtime, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt = EKF('GT');
            gt.setState(initState);
            gt.predict(sysModel);
            [gtMean, gtCov] = gt.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, [2 * ones(4, 1) gtMean]);
            obj.verifyEqual(stateCovs, cat(3, 2 * eye(4), gtCov));
        end
        
        function testPredictSingleOutOfRangeIndex(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(2 * ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.predictSingle(3, sysModel), 'FilterSet:NoFilter');
        end
        
        function testPredictSingleNoName(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(2 * ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.predictSingle('C', sysModel), 'FilterSet:NoFilter');
        end
        
        function testPredictSingleIdZero(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(2 * ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.predictSingle(0, sysModel), 'FilterSet:InvalidIdentifier');
        end
        
        function testPredictSingleInvalidId(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(2 * ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.predictSingle(ones(3), sysModel), 'FilterSet:InvalidIdentifier');
            obj.verifyError(@() set.predictSingle(Gaussian(), sysModel), 'FilterSet:InvalidIdentifier');
        end
        
        
        function testUpdate(obj)
            set = FilterSet();
            f   = EKF('KF');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.setStates(initState);
            runtimes = set.update(measModel, meas);
            
            obj.verifySize(runtimes, [1 1]);
            obj.verifyGreaterThan(runtimes, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt = EKF('GT');
            gt.setState(initState);
            gt.update(measModel, meas);
            [gtMean, gtCov] = gt.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, gtMean);
            obj.verifyEqual(stateCovs, gtCov);
        end
        
        function testUpdateMultiple(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState1 = Gaussian(ones(4, 1), 2 * eye(4));
            initState2 = Gaussian(3 * ones(4, 1), 4 * eye(4));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState1); % To set joint state dim
            
            f.setState(initState1);
            f2.setState(initState2);
            runtimes = set.update(measModel, meas);
            
            obj.verifySize(runtimes, [1 2]);
            obj.verifyGreaterThan(runtimes, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt1 = EKF('GT1');
            gt1.setState(initState1);
            gt1.update(measModel, meas);
            [gtMean1, gtCov1] = gt1.getStateMeanAndCov();
            
            gt2 = EKF('GT2');
            gt2.setState(initState2);
            gt2.update(measModel, meas);
            [gtMean2, gtCov2] = gt2.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, [gtMean1 gtMean2]);
            obj.verifyEqual(stateCovs, cat(3, gtCov1, gtCov2));
        end
        
        
        function testUpdateSingleName(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            runtime = set.updateSingle('KF2', measModel, meas);
            
            obj.verifySize(runtime, [1 1]);
            obj.verifyGreaterThan(runtime, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt = EKF('GT');
            gt.setState(initState);
            gt.update(measModel, meas);
            [gtMean, gtCov] = gt.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, [ones(4, 1) gtMean]);
            obj.verifyEqual(stateCovs, cat(3, 2 * eye(4), gtCov));
        end
        
        function testUpdateSingleId(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            runtime = set.updateSingle(2, measModel, meas);
            
            obj.verifySize(runtime, [1 1]);
            obj.verifyGreaterThan(runtime, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt = EKF('GT');
            gt.setState(initState);
            gt.update(measModel, meas);
            [gtMean, gtCov] = gt.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, [ones(4, 1) gtMean]);
            obj.verifyEqual(stateCovs, cat(3, 2 * eye(4), gtCov));
        end
        
        function testUpdateSingleOutOfRangeIndex(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.updateSingle(3, measModel, meas), 'FilterSet:NoFilter');
        end
        
        function testUpdateSingleNoName(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.updateSingle('C', measModel, meas), 'FilterSet:NoFilter');
        end
        
        function testUpdateSingleIdZero(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.updateSingle(0, measModel, meas), 'FilterSet:InvalidIdentifier');
        end
        
        function testUpdateSingleInvalidId(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.updateSingle(ones(3), measModel, meas), 'FilterSet:InvalidIdentifier');
            obj.verifyError(@() set.updateSingle(Gaussian(), measModel, meas), 'FilterSet:InvalidIdentifier');
        end
        
        
        function testStep(obj)
            set = FilterSet();
            f   = EKF('KF');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.setStates(initState);
            runtimes = set.step(sysModel, measModel, meas);
            
            obj.verifySize(runtimes, [1 1]);
            obj.verifyGreaterThan(runtimes, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt = EKF('GT');
            gt.setState(initState);
            gt.predict(sysModel);
            gt.update(measModel, meas);
            [gtMean, gtCov] = gt.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, gtMean);
            obj.verifyEqual(stateCovs, gtCov);
        end
        
        function testStepMultiple(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState1 = Gaussian(ones(4, 1), 2 * eye(4));
            initState2 = Gaussian(3 * ones(4, 1), 4 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState1); % To set joint state dim
            
            f.setState(initState1);
            f2.setState(initState2);
            runtimes = set.step(sysModel, measModel, meas);
            
            obj.verifySize(runtimes, [1 2]);
            obj.verifyGreaterThan(runtimes, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt1 = EKF('GT1');
            gt1.setState(initState1);
            gt1.predict(sysModel);
            gt1.update(measModel, meas);
            [gtMean1, gtCov1] = gt1.getStateMeanAndCov();
            
            gt2 = EKF('GT2');
            gt2.setState(initState2);
            gt2.predict(sysModel);
            gt2.update(measModel, meas);
            [gtMean2, gtCov2] = gt2.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, [gtMean1 gtMean2]);
            obj.verifyEqual(stateCovs, cat(3, gtCov1, gtCov2));
        end
        
        
        function testStepSingleName(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            runtime = set.stepSingle('KF2', sysModel, measModel, meas);
            
            obj.verifySize(runtime, [1 1]);
            obj.verifyGreaterThan(runtime, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt = EKF('GT');
            gt.setState(initState);
            gt.predict(sysModel);
            gt.update(measModel, meas);
            [gtMean, gtCov] = gt.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, [ones(4, 1) gtMean]);
            obj.verifyEqual(stateCovs, cat(3, 2 * eye(4), gtCov));
        end
        
        function testStepSingleId(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            runtime = set.stepSingle(2, sysModel, measModel, meas);
            
            obj.verifySize(runtime, [1 1]);
            obj.verifyGreaterThan(runtime, 0);
            
            [stateMeans, stateCovs] = set.getStatesMeanAndCov();
            
            gt = EKF('GT');
            gt.setState(initState);
            gt.predict(sysModel);
            gt.update(measModel, meas);
            [gtMean, gtCov] = gt.getStateMeanAndCov();
            
            obj.verifyEqual(stateMeans, [ones(4, 1) gtMean]);
            obj.verifyEqual(stateCovs, cat(3, 2 * eye(4), gtCov));
        end
        
        function testStepSingleOutOfRangeIndex(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.stepSingle(3, sysModel, measModel, meas), 'FilterSet:NoFilter');
        end
        
        function testStepSingleNoName(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.stepSingle('C', sysModel, measModel, meas), 'FilterSet:NoFilter');
        end
        
        function testStepSingleIdZero(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.stepSingle(0, sysModel, measModel, meas), 'FilterSet:InvalidIdentifier');
        end
        
        function testStepSingleInvalidId(obj)
            set = FilterSet();
            f   = EKF('KF');
            f2  = EKF('KF2');
            
            initState = Gaussian(ones(4, 1), 2 * eye(4));
            
            sysModel = LinearSystemModel(2 * ones(4));
            sysModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            
            measModel = LinearMeasurementModel(2 * ones(4));
            measModel.setNoise(Gaussian(zeros(4, 1), 5 * eye(4)));
            meas = [1 2 3 4]';
            
            set.add(f);
            set.add(f2);
            set.setStates(initState);
            
            obj.verifyError(@() set.stepSingle(ones(3), sysModel, measModel, meas), 'FilterSet:InvalidIdentifier');
            obj.verifyError(@() set.stepSingle(Gaussian(), sysModel, measModel, meas), 'FilterSet:InvalidIdentifier');
        end
    end
end
