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

#ifndef _TEST_UTILS_H_
#define _TEST_UTILS_H_

#include <Mex/Mex.h>

class TestUtils {
    public:
        static std::string MissedExName() {
            return std::string("Missed expected exception.");
        }
        
        static void checkException(const std::exception& ex,
                                   const std::string& expMsg) {
            const std::string recvMsg = ex.what();
            
            if (recvMsg == MissedExName()) {
                throw std::runtime_error(MissedExName() + "\nExpected: " + expMsg);
            }
            
            if (expMsg != recvMsg) {
                throw std::runtime_error("Unexpected exception.\nExpected: " + expMsg + "\nGot     : " + recvMsg);
            }
        }
        
        template<typename Matrix>
        static void checkSize(const Matrix& mat,
                              int64_t rows,
                              int64_t cols) {
            if (mat.rows() != rows) {
                throw std::runtime_error("Matrix with unexpected number of rows.");
            }
            
            if (mat.cols() != cols) {
                throw std::runtime_error("Matrix with unexpected number of columns.");
            }
        }
        
        template<typename Matrix>
        static void checkSize3D(const Matrix& mat,
                                int64_t rows,
                                int64_t cols) {
            const Mex::Dimensions& dims = mat.dims();
            const Mex::Dimensions& slices = mat.slices();
            
            if (dims.size() != 2) {
                throw std::runtime_error("Matrix with unexpected number of dimensions.");
            }
            
            if (slices.size() != 0) {
                throw std::runtime_error("Matrix with unexpected number of slices.");
            }
            
            if (dims(0) != rows || mat.dim(0) != rows || mat.rows() != rows) {
                throw std::runtime_error("Matrix with unexpected number of rows.");
            }
            
            if (dims(1) != cols || mat.dim(1) != cols || mat.cols() != cols) {
                throw std::runtime_error("Matrix with unexpected number of columns.");
            }
            
            if (mat.dim(2) != 1) {
                throw std::runtime_error("Matrix with unexpected number of slice dimension 1.");
            }
            
            if (mat.size() != (rows * cols)) {
                throw std::runtime_error("Matrix with unexpected size.");
            }
        }
        
        template<typename Matrix>
        static void checkSize3D(const Matrix& mat,
                                int64_t rows,
                                int64_t cols,
                                int64_t firstSlice) {
            const Mex::Dimensions& dims   = mat.dims();
            const Mex::Dimensions& slices = mat.slices();
            
            if (dims.size() != 3) {
                throw std::runtime_error("Matrix with unexpected number of dimensions.");
            }
            
            if (slices.size() != 1) {
                throw std::runtime_error("Matrix with unexpected number of slices.");
            }
            
            if (dims(0) != rows || mat.dim(0) != rows || mat.rows() != rows) {
                throw std::runtime_error("Matrix with unexpected number of rows.");
            }
            
            if (dims(1) != cols || mat.dim(1) != cols || mat.cols() != cols) {
                throw std::runtime_error("Matrix with unexpected number of columns.");
            }
            
            if (dims(2) != firstSlice || slices(0) != firstSlice || mat.dim(2) != firstSlice) {
                throw std::runtime_error("Matrix with unexpected number of slice dimension 1.");
            }
            
            if (mat.dim(3) != 1) {
                throw std::runtime_error("Matrix with unexpected number of slice dimension 2.");
            }
            
            if (mat.size() != (rows * cols * firstSlice)) {
                throw std::runtime_error("Matrix with unexpected size.");
            }
        }
        
        template<typename Matrix, typename Scalar>
        static void checkContent(const Matrix& mat,
                                 const Scalar& value) {
            const int64_t n = mat.size();
            
            for (int64_t i = 0; i < n; ++i) {
                if (mat(i) != value) {
                    throw std::runtime_error("Matrix with unexpected content.");
                }
            }
        }
        
        
        template<typename Slice, typename Scalar>
        static void checkSlice(Slice& slice,
                               const Scalar start,
                               const int64_t num) {
            if (slice.size() != num) {
                throw std::runtime_error("Slice with unexpected size.");
            }
            
            Scalar val = start;
            
            for (int64_t i = 0; i < num; ++i) {
                if (slice(i) != val) {
                    throw std::runtime_error("Slice with unexpected content.");
                }
                
                ++val;
            }
        }
        
        template<typename Scalar>
        static void setArray(mxArray* array,
                             const Scalar& val) {
            const mwSize n = mxGetNumberOfElements(array);
            Scalar* data = (Scalar*)mxGetData(array);
            
            for (mwSize i = 0; i < n; ++i) {
                data[i] = val;
            }
        }
        
        template<typename Scalar>
        static void setArray(mxArray* array) {
            const mwSize n = mxGetNumberOfElements(array);
            Scalar* data = (Scalar*)mxGetData(array);
            
            for (mwSize i = 0; i < n; ++i) {
                data[i] = (Scalar)i;
            }
        }
        
        template<typename Matrix, typename Scalar>
        static void setMatrix(Matrix& mat) {
            const int64_t n = mat.size();
            
            for (int64_t i = 0; i < n; ++i) {
                mat(i) = (Scalar)i;
            }
        }
        
};

#endif
