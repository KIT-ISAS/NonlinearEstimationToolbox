/* >> This file is part of the Nonlinear Estimation Toolbox
 *
 *    For more information, see https://bitbucket.org/nonlinearestimation/toolbox
 *
 *    Copyright (C) 2015-2017  Jannik Steinbring <nonlinearestimation@gmail.com>
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "TestUtils.h"

template<typename Scalar>
class TestMatrix {
    public:
        static void run() {
            TestMatrix<Scalar> test;
            
            test.testValidVec();
            test.testInvalidVec();
            
            test.testValidMat();
            test.testInvalidMat();
            
            test.testValidArray();
            test.testInvalidTypeArray();
            test.testInvalidSizeArray();
            test.testInvalidDimArray();
            
            test.testValidMatrix();
            test.testInvalidMatrix();
            
            test.testValidEigenMat();
            test.testInvalidEigenMat();
        }
        
        void testValidVec() {
            Mex::Matrix<Scalar, 1, 3> v1(3);
            v1.setOnes();
            checkRowVector(v1);
            
            Mex::Matrix<Scalar, 1, Eigen::Dynamic> v2(3);
            v2.setOnes();
            checkRowVector(v2);
            
            Mex::Matrix<Scalar, 1, 3> v3;
            v3.setOnes();
            checkRowVector(v3);
            
            Mex::Matrix<Scalar, 2, 1> v4(2);
            v4.setOnes();
            checkVector(v4);
            
            Mex::Matrix<Scalar, Eigen::Dynamic, 1> v5(2);
            v5.setOnes();
            checkVector(v5);
            
            Mex::Matrix<Scalar, 2, 1> v6;
            v6.setOnes();
            checkVector(v6);
        }
        
        void testInvalidVec() {
            try {
                Mex::Matrix<Scalar, 1, 3> m(4);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 3, 1> m(4);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
        }
        
        
        
        void testValidMat() {
            Mex::Matrix<Scalar, 2, 3> m1(2, 3);
            m1.setOnes();
            checkMatrix(m1);
            
            Mex::Matrix<Scalar, 2, Eigen::Dynamic> m2(2, 3);
            m2.setOnes();
            checkMatrix(m2);
            
            Mex::Matrix<Scalar, Eigen::Dynamic, 3> m3(2, 3);
            m3.setOnes();
            checkMatrix(m3);
            
            Mex::MatrixX<Scalar> m4(2, 3);
            m4.setOnes();
            checkMatrix(m4);
            
            Mex::Matrix<Scalar, 2, 3> m5;
            m5.setOnes();
            checkMatrix(m5);
        }
        
        void testInvalidMat() {
            try {
                Mex::Matrix<Scalar, 2, 3> m(4, 3);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 2, Eigen::Dynamic> m(4, 3);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 2, 3> m(2, 4);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, Eigen::Dynamic, 3> m(2, 4);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
        }
        
        
        
        void testValidArray() {
            Mex::Matrix<Scalar, 2, 3> m1(createArray());
            m1.setOnes();
            checkMatrix(m1);
            
            Mex::Matrix<Scalar, 2, Eigen::Dynamic> m2(createArray());
            m2.setOnes();
            checkMatrix(m2);
            
            Mex::Matrix<Scalar, Eigen::Dynamic, 3> m3(createArray());
            m3.setOnes();
            checkMatrix(m3);
            
            Mex::MatrixX<Scalar> m4(createArray());
            m4.setOnes();
            checkMatrix(m4);
        }
        
        void testInvalidTypeArray() {
            const mwSize trueDims[2] = { 2, 3 };
            
            mxArray* invalidTypeArray = mxCreateNumericArray(2, trueDims, mxCHAR_CLASS, mxREAL);
            
            try {
                Mex::Matrix<Scalar, 2, 3> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 2, Eigen::Dynamic> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, Eigen::Dynamic, 3> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::MatrixX<Scalar> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidTypeArray);
        }
        
        void testInvalidSizeArray() {
            const mwSize invalidDims[3] = { 4, 5, 3 };
            
            mxArray* invalidArray = mxCreateNumericArray(3, invalidDims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            try {
                Mex::Matrix<Scalar, 2, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array is not a matrix.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 2, Eigen::Dynamic> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array is not a matrix.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, Eigen::Dynamic, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array is not a matrix.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::MatrixX<Scalar> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array is not a matrix.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidArray);
        }
        
        void testInvalidDimArray() {
            mwSize invalidDims[2] = { 4, 3 };
            
            mxArray* invalidArray = mxCreateNumericArray(2, invalidDims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            try {
                Mex::Matrix<Scalar, 2, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 2, Eigen::Dynamic> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidArray);
            
            invalidDims[0] = 2;
            invalidDims[1] = 5;
            
            invalidArray = mxCreateNumericArray(2, invalidDims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            try {
                Mex::Matrix<Scalar, 2, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (5) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, Eigen::Dynamic, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (5) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidArray);
        }
        
        
        
        void testValidMatrix() {
            Mex::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> mat(2, 3);
            mat.setOnes();
            
            Mex::Matrix<Scalar, 2, 3> m1(mat);
            checkMatrix(m1);
            
            Mex::Matrix<Scalar, 2, Eigen::Dynamic> m2(mat);
            checkMatrix(m2);
            
            Mex::Matrix<Scalar, Eigen::Dynamic, 3> m3(mat);
            checkMatrix(m3);
            
            Mex::MatrixX<Scalar> m4(mat);
            checkMatrix(m4);
        }
        
        void testInvalidMatrix() {
            Mex::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> mat(2, 3);
            mat.setOnes();
            
            try {
                Mex::Matrix<Scalar, 4, 3> m(mat);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (2) and expected (4) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 4, Eigen::Dynamic> m(mat);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (2) and expected (4) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 2, 5> m(mat);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (3) and expected (5) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, Eigen::Dynamic, 5> m(mat);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (3) and expected (5) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
        }
        
        
        
        void testValidEigenMat() {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> eigenMat(2, 3);
            eigenMat.setOnes();
            
            Mex::Matrix<Scalar, 2, 3> m1(eigenMat);
            checkMatrix(m1);
            
            Mex::Matrix<Scalar, 2, Eigen::Dynamic> m2(eigenMat);
            checkMatrix(m2);
            
            Mex::Matrix<Scalar, Eigen::Dynamic, 3> m3(eigenMat);
            checkMatrix(m3);
            
            Mex::MatrixX<Scalar> m4(eigenMat);
            checkMatrix(m4);
        }
        
        void testInvalidEigenMat() {
            Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> eigenMat(2, 3);
            eigenMat.setOnes();
            
            try {
                Mex::Matrix<Scalar, 4, 3> m(eigenMat);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (2) and expected (4) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 4, Eigen::Dynamic> m(eigenMat);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (2) and expected (4) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, 2, 5> m(eigenMat);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (3) and expected (5) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::Matrix<Scalar, Eigen::Dynamic, 5> m(eigenMat);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (3) and expected (5) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
        }
        
    private:
        mxArray* createArray() {
            const mwSize dims[2] = { 2, 3 };
            
            mxArray* array = mxCreateNumericArray(2, dims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            TestUtils::setArray<Scalar>(array, 1);
            
            return array;
        }
        
        template<typename Derived>
        void checkMatrix(const Eigen::MatrixBase<Derived>& mat) {
            TestUtils::checkSize(mat, 2, 3);
            TestUtils::checkContent<Eigen::MatrixBase<Derived>, Scalar>(mat, 1);
        }
        
        template<typename Derived>
        void checkRowVector(const Eigen::MatrixBase<Derived>& mat) {
            TestUtils::checkSize(mat, 1, 3);
            TestUtils::checkContent<Eigen::MatrixBase<Derived>, Scalar>(mat, 1);
        }
        
        template<typename Derived>
        void checkVector(const Eigen::MatrixBase<Derived>& mat) {
            TestUtils::checkSize(mat, 2, 1);
            TestUtils::checkContent<Eigen::MatrixBase<Derived>, Scalar>(mat, 1);
        }
        
};

void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
    try {
        TestMatrix<float>::run();
        TestMatrix<double>::run();
        
        TestMatrix<int8_t>::run();
        TestMatrix<uint8_t>::run();
        
        TestMatrix<int16_t>::run();
        TestMatrix<uint16_t>::run();
        
        TestMatrix<int32_t>::run();
        TestMatrix<uint32_t>::run();
        
        TestMatrix<int64_t>::run();
        TestMatrix<uint64_t>::run();
    } catch (std::exception& ex) {
        std::stringstream ss;
        
        ss << "Unit test failed: " << ex.what();
        
        mexErrMsgTxt(ss.str().c_str());
    }
}
