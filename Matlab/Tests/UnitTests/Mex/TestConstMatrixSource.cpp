
#include "TestUtils.h"

template<typename Scalar>
class TestConstMatrix {
    public:
        static void run() {
            TestConstMatrix<Scalar> test;
            
            test.testValidArray();
            test.testInvalidTypeArray();
            test.testInvalidSizeArray();
            test.testInvalidDimArray();
        }
        
        TestConstMatrix() {
            const mwSize dims[2] = { 2, 3 };
            
            array = mxCreateNumericArray(2, dims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            TestUtils::setArray<Scalar>(array, 105);
        }
        
        ~TestConstMatrix() { 
            mxDestroyArray(array);
        }
        
        void testValidArray() {
            Mex::ConstMatrix<Scalar, 2, 3> m1(array);
            checkMatrix(m1);
            
            Mex::ConstMatrix<Scalar, 2, Eigen::Dynamic> m2(array);
            checkMatrix(m2);
            
            Mex::ConstMatrix<Scalar, Eigen::Dynamic, 3> m3(array);
            checkMatrix(m3);
            
            Mex::ConstMatrixX<Scalar> m4(array);
            checkMatrix(m4);
        }
        
        void testInvalidTypeArray() {
            const mwSize dims[2] = { 2, 3 };
            
            mxArray* invalidTypeArray = mxCreateNumericArray(2, dims, mxCHAR_CLASS, mxREAL);
            
            try {
                Mex::ConstMatrix<Scalar, 2, 3> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrix<Scalar, 2, Eigen::Dynamic> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrix<Scalar, Eigen::Dynamic, 3> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrixX<Scalar> m(invalidTypeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array of invalid type.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidTypeArray);
        }
        
        void testInvalidSizeArray() {
            const mwSize invalidDims[3] = { 4, 5, 3 };
            
            mxArray* invalidSizeArray = mxCreateNumericArray(3, invalidDims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            try {
                Mex::ConstMatrix<Scalar, 2, 3> m(invalidSizeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array is not a matrix.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrix<Scalar, 2, Eigen::Dynamic> m(invalidSizeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array is not a matrix.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrix<Scalar, Eigen::Dynamic, 3> m(invalidSizeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array is not a matrix.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrixX<Scalar> m(invalidSizeArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("MX array is not a matrix.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidSizeArray);
        }
        
        void testInvalidDimArray() {
            mwSize invalidDims[2] = { 4, 3 };
            
            mxArray* invalidArray = mxCreateNumericArray(2, invalidDims, Mex::Traits<Scalar>::MxClassID, mxREAL);
            
            try {
                Mex::ConstMatrix<Scalar, 2, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (4) and expected (2) number of rows.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrix<Scalar, 2, Eigen::Dynamic> m(invalidArray);
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
                Mex::ConstMatrix<Scalar, 2, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (5) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            try {
                Mex::ConstMatrix<Scalar, Eigen::Dynamic, 3> m(invalidArray);
                throw std::runtime_error(TestUtils::MissedExName());
            } catch (std::exception& ex) {
                const std::string expMsg("Mismatch between given (5) and expected (3) number of columns.");
                
                TestUtils::checkException(ex, expMsg);
            }
            
            mxDestroyArray(invalidArray);
        }
        
    private:
        template<typename Derived>
        void checkMatrix(const Eigen::MatrixBase<Derived>& mat) {
            TestUtils::checkSize(mat, 2, 3);
            TestUtils::checkContent<Eigen::MatrixBase<Derived>, Scalar>(mat, 105);
        }
        
    private:
        mxArray* array;
        
};

void mexFunction(int numOutputs, mxArray *outputs[],
                 int numInputs, const mxArray *inputs[])
{
    try {
        TestConstMatrix<float>::run();
        TestConstMatrix<double>::run();
        
        TestConstMatrix<int8_t>::run();
        TestConstMatrix<uint8_t>::run();
        
        TestConstMatrix<int16_t>::run();
        TestConstMatrix<uint16_t>::run();
        
        TestConstMatrix<int32_t>::run();
        TestConstMatrix<uint32_t>::run();
        
        TestConstMatrix<int64_t>::run();
        TestConstMatrix<uint64_t>::run();
    } catch (std::exception& ex) {
        std::stringstream ss;
        
        ss << "Unit test failed: " << ex.what();
        
        mexErrMsgTxt(ss.str().c_str());
    }
}
