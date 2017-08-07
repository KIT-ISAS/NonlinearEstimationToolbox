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
class TestOutputMatrixXD {
    public:
        static void run() {
            TestOutputMatrixXD<Scalar> test;
            
            test.testValidList2();
            test.testValidList3();
            test.testInvalidList1();
            test.testInvalidList();
            
            test.testValidDims2();
            test.testValidDims3();
            test.testInvalidDims1();
            test.testInvalidDims();
            
            test.testValidDim2();
            test.testValidDim3();
            test.testInvalidDim();
            
            test.testValidArray2();
            test.testValidArray3();
            test.testInvalidTypeArray();
            test.testInvalidDimArray();
            
            test.testSlice();
            test.testConstSlice();
        }
        
        void testValidList2() {
            Mex::OutputMatrixXD<Scalar, 2, 3> m1{2, 3};
            m1.setOnes();
            checkMatrix2(m1);
            mxDestroyArray(m1);
            
            Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m2{2, 3};
            m2.setOnes();
            checkMatrix2(m2);
            mxDestroyArray(m2);
            
            Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m3{2, 3};
            m3.setOnes();
            checkMatrix2(m3);
            mxDestroyArray(m3);
            
            Mex::OutputMatrixXDX<Scalar> m4{2, 3};
            m4.setOnes();
            checkMatrix2(m4);
            mxDestroyArray(m4);
        }
        
        void testValidList3() {
            Mex::OutputMatrixXD<Scalar, 2, 3> m1{2, 3, 5};
            m1.setOnes();
            checkMatrix3(m1);
            mxDestroyArray(m1);
            
            Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m2{2, 3, 5};
            m2.setOnes();
            checkMatrix3(m2);
            mxDestroyArray(m2);
            
            Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m3{2, 3, 5};
            m3.setOnes();
            checkMatrix3(m3);
            mxDestroyArray(m3);
            
            Mex::OutputMatrixXDX<Scalar> m4{2, 3, 5};
            m4.setOnes();
            checkMatrix3(m4);
            mxDestroyArray(m4);
        }
        
        void testInvalidList1() {
            try {
                Mex::OutputMatrixXD<Scalar, 2, 3> m{2};
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("At least two dimensions must be provided.");
                
                TestUtils::checkException(ex, expMsg);
            }
        }
        
        void testInvalidList() {
            try {
                Mex::OutputMatrixXD<Scalar, 2, 3> m{4, 3};
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m{4, 3};
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, 3> m{2, 4};
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m{2, 4};
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
        }
        
        
        
        void testValidDims2() {
            Mex::Dimensions dims(2);
            dims(0) = 2;
            dims(1) = 3;
            
            Mex::OutputMatrixXD<Scalar, 2, 3> m1(dims);
            m1.setOnes();
            checkMatrix2(m1);
            mxDestroyArray(m1);
            
            Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m2(dims);
            m2.setOnes();
            checkMatrix2(m2);
            mxDestroyArray(m2);
            
            Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m3(dims);
            m3.setOnes();
            checkMatrix2(m3);
            mxDestroyArray(m3);
            
            Mex::OutputMatrixXDX<Scalar> m4(dims);
            m4.setOnes();
            checkMatrix2(m4);
            mxDestroyArray(m4);
        }
        
        void testValidDims3() {
            Mex::Dimensions dims(3);
            dims(0) = 2;
            dims(1) = 3;
            dims(2) = 5;
            
            Mex::OutputMatrixXD<Scalar, 2, 3> m1(dims);
            m1.setOnes();
            checkMatrix3(m1);
            mxDestroyArray(m1);
            
            Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m2(dims);
            m2.setOnes();
            checkMatrix3(m2);
            mxDestroyArray(m2);
            
            Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m3(dims);
            m3.setOnes();
            checkMatrix3(m3);
            mxDestroyArray(m3);
            
            Mex::OutputMatrixXDX<Scalar> m4(dims);
            m4.setOnes();
            checkMatrix3(m4);
            mxDestroyArray(m4);
        }
        
        void testInvalidDims1() {
            try {
                Mex::Dimensions dims(1);
                dims(0) = 2;
                
                Mex::OutputMatrixXD<Scalar, 2, 3> m(dims);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("At least two dimensions must be provided.");
                
                TestUtils::checkException(ex, expMsg);
            }
        }
        
        void testInvalidDims() {
            Mex::Dimensions dims(2);
            dims(0) = 4;
            dims(1) = 3;
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, 3> m(dims);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m(dims);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            dims(0) = 2;
            dims(1) = 4;
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, 3> m(dims);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m(dims);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
        }
        
        
        
        void testValidDim2() {
            Mex::Dimensions dims;
            
            Mex::OutputMatrixXD<Scalar, 2, 3> m1(2, 3, dims);
            m1.setOnes();
            checkMatrix2(m1);
            mxDestroyArray(m1);
            
            Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m2(2, 3, dims);
            m2.setOnes();
            checkMatrix2(m2);
            mxDestroyArray(m2);
            
            Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m3(2, 3, dims);
            m3.setOnes();
            checkMatrix2(m3);
            mxDestroyArray(m3);
            
            Mex::OutputMatrixXDX<Scalar> m4(2, 3, dims);
            m4.setOnes();
            checkMatrix2(m4);
            mxDestroyArray(m4);
        }
        
        void testValidDim3() {
            Mex::Dimensions dims(1);
            dims(0) = 5;
            
            Mex::OutputMatrixXD<Scalar, 2, 3> m1(2, 3, dims);
            m1.setOnes();
            checkMatrix3(m1);
            mxDestroyArray(m1);
            
            Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m2(2, 3, dims);
            m2.setOnes();
            checkMatrix3(m2);
            mxDestroyArray(m2);
            
            Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m3(2, 3, dims);
            m3.setOnes();
            checkMatrix3(m3);
            mxDestroyArray(m3);
            
            Mex::OutputMatrixXDX<Scalar> m4(2, 3, dims);
            m4.setOnes();
            checkMatrix3(m4);
            mxDestroyArray(m4);
        }
        
        void testInvalidDim() {
            Mex::Dimensions dims;
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, 3> m(4, 3, dims);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m(4, 3, dims);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, 3> m(2, 4, dims);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m(2, 4, dims);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
        }
        
        
        
        void testValidArray2() {
            const mwSize dims[2] = { 2, 3 };
            
            mxArray* array2 = mxCreateNumericArray(2, dims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            TestUtils::setArray<Scalar>(array2, 1);
            
            Mex::OutputMatrixXD<Scalar, 2, 3> m1(array2);
            checkMatrix2(m1);
            
            Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m2(array2);
            checkMatrix2(m2);
            
            Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m3(array2);
            checkMatrix2(m3);
            
            Mex::OutputMatrixXDX<Scalar> m4(array2);
            checkMatrix2(m4);
            
            mxDestroyArray(array2);
        }
        
        void testValidArray3() {
            const mwSize dims[3] = { 2, 3, 5 };
            
            mxArray* array3 = mxCreateNumericArray(3, dims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            TestUtils::setArray<Scalar>(array3, 1);
            
            Mex::OutputMatrixXD<Scalar, 2, 3> m1(array3);
            checkMatrix3(m1);
            
            Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m2(array3);
            checkMatrix3(m2);
            
            Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m3(array3);
            checkMatrix3(m3);
            
            Mex::OutputMatrixXDX<Scalar> m4(array3);
            checkMatrix3(m4);
            
            mxDestroyArray(array3);
        }
        
        void testInvalidTypeArray() {
            const mwSize trueDims[3] = { 2, 3, 5 };
            
            mxArray* invalidTypeArray = mxCreateNumericArray(3, trueDims, mxCHAR_CLASS, mxREAL);
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, 3> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXDX<Scalar> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidTypeArray);
        }
        
        void testInvalidDimArray() {
            mwSize invalidDims[2] = { 4, 3 };
            
            mxArray* invalidArray = mxCreateNumericArray(2, invalidDims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, 2, Eigen::Dynamic> m(invalidArray);
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
                Mex::OutputMatrixXD<Scalar, 2, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (5) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::OutputMatrixXD<Scalar, Eigen::Dynamic, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (5) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidArray);
        }
        
        
        
        void testSlice() {
            typedef Mex::OutputMatrixXD<Scalar, 3, 4> Matrix;
            
            Matrix mat{3, 4, 2};
            
            TestUtils::setMatrix<Matrix, Scalar>(mat);
            
            typename Matrix::Slice s0 = mat.slice(0);
            typename Matrix::Slice s1 = mat.slice(1);
            
            TestUtils::checkSlice<typename Matrix::Slice, Scalar>(s0, 0, 3 * 4);
            TestUtils::checkSlice<typename Matrix::Slice, Scalar>(s1, 3 * 4, 3 * 4);
            
            Mex::Dimensions d(1);
            
            d(0) = 0;
            typename Matrix::Slice s2 = mat.slice(d);
            
            d(0) = 1;
            typename Matrix::Slice s3 = mat.slice(d);
            
            TestUtils::checkSlice<typename Matrix::Slice, Scalar>(s2, 0, 3 * 4);
            TestUtils::checkSlice<typename Matrix::Slice, Scalar>(s3, 3 * 4, 3 * 4);
            
            mxDestroyArray(mat);
        }
        
        void testConstSlice() {
            typedef Mex::OutputMatrixXD<Scalar, 3, 4> Matrix;
            
            Matrix mat{3, 4, 2};
            
            TestUtils::setMatrix<Matrix, Scalar>(mat);
            
            typename Matrix::ConstSlice s0 = ((const Matrix&)mat).slice(0);
            typename Matrix::ConstSlice s1 = ((const Matrix&)mat).slice(1);
            
            TestUtils::checkSlice<typename Matrix::ConstSlice, Scalar>(s0, 0, 3 * 4);
            TestUtils::checkSlice<typename Matrix::ConstSlice, Scalar>(s1, 3 * 4, 3 * 4);
            
            Mex::Dimensions d(1);
            
            d(0) = 0;
            typename Matrix::ConstSlice s2 = ((const Matrix&)mat).slice(d);
            
            d(0) = 1;
            typename Matrix::ConstSlice s3 = ((const Matrix&)mat).slice(d);
            
            TestUtils::checkSlice<typename Matrix::ConstSlice, Scalar>(s2, 0, 3 * 4);
            TestUtils::checkSlice<typename Matrix::ConstSlice, Scalar>(s3, 3 * 4, 3 * 4);
            
            mxDestroyArray(mat);
        }
        
    private:
        template<typename Matrix>
        void checkMatrix2(const Matrix& mat) {
            TestUtils::checkSize3D(mat, 2, 3);
            TestUtils::checkContent<Matrix, Scalar>(mat, 1);
        }
        
        template<typename Matrix>
        void checkMatrix3(const Matrix& mat) {
            TestUtils::checkSize3D(mat, 2, 3, 5);
            TestUtils::checkContent<Matrix, Scalar>(mat, 1);
        }
        
};

void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
    try {
        TestOutputMatrixXD<float>::run();
        TestOutputMatrixXD<double>::run();
        
        TestOutputMatrixXD<int8_t>::run();
        TestOutputMatrixXD<uint8_t>::run();
        
        TestOutputMatrixXD<int16_t>::run();
        TestOutputMatrixXD<uint16_t>::run();
        
        TestOutputMatrixXD<int32_t>::run();
        TestOutputMatrixXD<uint32_t>::run();
        
        TestOutputMatrixXD<int64_t>::run();
        TestOutputMatrixXD<uint64_t>::run();
    } catch (std::exception& ex) {
        std::stringstream ss;
        
        ss << "Unit test failed: " << ex.what();
        
        mexErrMsgTxt(ss.str().c_str());
    }
}
