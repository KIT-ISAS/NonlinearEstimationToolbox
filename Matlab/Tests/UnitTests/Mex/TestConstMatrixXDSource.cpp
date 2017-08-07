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
class TestConstMatrixXD {
    public:
        static void run() {
            TestConstMatrixXD<Scalar> test;
            
            test.testValidArray2();
            test.testValidArray3();
            test.testInvalidTypeArray();
            test.testInvalidDimArray();
            
            test.testConstSlice();
        }
        
        TestConstMatrixXD() {
            const mwSize dims2[2] = { 2, 3 };
            
            array2 = mxCreateNumericArray(2, dims2, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            TestUtils::setArray<Scalar>(array2, 105);
            
            const mwSize dims3[3] = { 2, 3, 5 };
            
            array3 = mxCreateNumericArray(3, dims3, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            TestUtils::setArray<Scalar>(array3, 105);
        }
        
        ~TestConstMatrixXD() {
            mxDestroyArray(array2);
            mxDestroyArray(array3);
        }
        
        void testValidArray2() {
            Mex::ConstMatrixXD<Scalar, 2, 3> m1(array2);
            checkMatrix2(m1);
            
            Mex::ConstMatrixXD<Scalar, 2, Eigen::Dynamic> m2(array2);
            checkMatrix2(m2);
            
            Mex::ConstMatrixXD<Scalar, Eigen::Dynamic, 3> m3(array2);
            checkMatrix2(m3);
            
            Mex::ConstMatrixXDX<Scalar> m4(array2);
            checkMatrix2(m4);
        }
        
        void testValidArray3() {
            Mex::ConstMatrixXD<Scalar, 2, 3> m1(array3);
            checkMatrix3(m1);
            
            Mex::ConstMatrixXD<Scalar, 2, Eigen::Dynamic> m2(array3);
            checkMatrix3(m2);
            
            Mex::ConstMatrixXD<Scalar, Eigen::Dynamic, 3> m3(array3);
            checkMatrix3(m3);
            
            Mex::ConstMatrixXDX<Scalar> m4(array3);
            checkMatrix3(m4);
        }
        
        void testInvalidTypeArray() {
            const mwSize dims[3] = { 2, 3, 5 };
            
            mxArray* invalidTypeArray = mxCreateNumericArray(3, dims, mxCHAR_CLASS, mxREAL);
            
            try {
                Mex::ConstMatrixXD<Scalar, 2, 3> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrixXD<Scalar, 2, Eigen::Dynamic> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrixXD<Scalar, Eigen::Dynamic, 3> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrixXDX<Scalar> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidTypeArray);
        }
        
       void testInvalidDimArray() {
            mwSize invalidDims[3] = { 4, 3, 6 };
            
            mxArray* invalidArray = mxCreateNumericArray(3, invalidDims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            try {
                Mex::ConstMatrixXD<Scalar, 2, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrixXD<Scalar, 2, Eigen::Dynamic> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidArray);
            
            invalidDims[0] = 2;
            invalidDims[1] = 5;
            
            invalidArray = mxCreateNumericArray(3, invalidDims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            try {
                Mex::ConstMatrixXD<Scalar, 2, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (5) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrixXD<Scalar, Eigen::Dynamic, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (5) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidArray);
        }
        
        
        
        void testConstSlice() {
            typedef Mex::ConstMatrixXD<Scalar, 3, 4> Matrix;
            
            const mwSize dims[3] = { 3, 4, 2 };
            
            mxArray* array = mxCreateNumericArray(3, dims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            TestUtils::setArray<Scalar>(array);
            
            Matrix mat(array);
            
            typename Matrix::ConstSlice s0 = mat.slice(0);
            typename Matrix::ConstSlice s1 = mat.slice(1);
            
            TestUtils::checkSlice<typename Matrix::ConstSlice, Scalar>(s0, 0, 3 * 4);
            TestUtils::checkSlice<typename Matrix::ConstSlice, Scalar>(s1, 3 * 4, 3 * 4);
            
            Mex::Dimensions d(1);
            
            d(0) = 0;
            typename Matrix::ConstSlice s2 = mat.slice(d);
            
            d(0) = 1;
            typename Matrix::ConstSlice s3 = mat.slice(d);
            
            TestUtils::checkSlice<typename Matrix::ConstSlice, Scalar>(s2, 0, 3 * 4);
            TestUtils::checkSlice<typename Matrix::ConstSlice, Scalar>(s3, 3 * 4, 3 * 4);
            
            mxDestroyArray(array);
        }
        
    private:
        template<typename Matrix>
        void checkMatrix2(const Matrix& mat) {
            TestUtils::checkSize3D(mat, 2, 3);
            TestUtils::checkContent<Matrix, Scalar>(mat, 105);
        }
        
        template<typename Matrix>
        void checkMatrix3(const Matrix& mat) {
            TestUtils::checkSize3D(mat, 2, 3, 5);
            TestUtils::checkContent<Matrix, Scalar>(mat, 105);
        }
        
    private:
        mxArray* array2;
        mxArray* array3;
        
};

void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
    try {
        TestConstMatrixXD<float>::run();
        TestConstMatrixXD<double>::run();
        
        TestConstMatrixXD<int8_t>::run();
        TestConstMatrixXD<uint8_t>::run();
        
        TestConstMatrixXD<int16_t>::run();
        TestConstMatrixXD<uint16_t>::run();
        
        TestConstMatrixXD<int32_t>::run();
        TestConstMatrixXD<uint32_t>::run();
        
        TestConstMatrixXD<int64_t>::run();
        TestConstMatrixXD<uint64_t>::run();
    } catch (std::exception& ex) {
        std::stringstream ss;
        
        ss << "Unit test failed: " << ex.what();
        
        mexErrMsgTxt(ss.str().c_str());
    }
}
