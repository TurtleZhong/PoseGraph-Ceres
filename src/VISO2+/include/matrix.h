/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

#ifndef MATRIX_H
#define MATRIX_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <vector>

#if defined(_MSC_VER) && (_MSC_VER < 1600)
  typedef __int8            int8_t;
  typedef __int16           int16_t;
  typedef __int32           int32_t;
  typedef __int64           int64_t;
  typedef unsigned __int8   uint8_t;
  typedef unsigned __int16  uint16_t;
  typedef unsigned __int32  uint32_t;
  typedef unsigned __int64  uint64_t;
#else
  #include <stdint.h>
#endif

#define endll endl << endl // double end line definition

typedef double FLOAT;      // double precision
//typedef float  FLOAT;    // single precision

class Matrix_ {

public:

  // constructor / deconstructor
  Matrix_ ();                                                  // init empty 0x0 matrix
  Matrix_ (const int32_t m,const int32_t n);                   // init empty mxn matrix
  Matrix_ (const int32_t m,const int32_t n,const FLOAT* val_); // init mxn matrix with values from array 'val'
  Matrix_ (const Matrix_ &M);                                   // creates deepcopy of M
  ~Matrix_ ();

  // assignment operator, copies contents of M
  Matrix_& operator= (const Matrix_ &M);

  // copies submatrix of M into array 'val', default values copy whole row/column/matrix
  void getData(FLOAT* val_,int32_t i1=0,int32_t j1=0,int32_t i2=-1,int32_t j2=-1);

  // set or get submatrices of current matrix
  Matrix_ getMat(int32_t i1,int32_t j1,int32_t i2=-1,int32_t j2=-1);
  void   setMat(const Matrix_ &M,const int32_t i,const int32_t j);

  // set sub-matrix to scalar (default 0), -1 as end replaces whole row/column/matrix
  void setVal(FLOAT s,int32_t i1=0,int32_t j1=0,int32_t i2=-1,int32_t j2=-1);

  // set (part of) diagonal to scalar, -1 as end replaces whole diagonal
  void setDiag(FLOAT s,int32_t i1=0,int32_t i2=-1);

  // clear matrix
  void zero();

  // extract columns with given index
  Matrix_ extractCols (std::vector<int> idx);

  // create identity matrix
  static Matrix_ eye (const int32_t m);
  void          eye ();

  // create diagonal matrix with nx1 or 1xn matrix M as elements
  static Matrix_ diag(const Matrix_ &M);

  // returns the m-by-n matrix whose elements are taken column-wise from M
  static Matrix_ reshape(const Matrix_ &M,int32_t m,int32_t n);

  // create 3x3 rotation matrices (convention: http://en.wikipedia.org/wiki/Rotation_matrix)
  static Matrix_ rotMatX(const FLOAT &angle);
  static Matrix_ rotMatY(const FLOAT &angle);
  static Matrix_ rotMatZ(const FLOAT &angle);

  // simple arithmetic operations
  Matrix_  operator+ (const Matrix_ &M); // add matrix
  Matrix_  operator- (const Matrix_ &M); // subtract matrix
  Matrix_  operator* (const Matrix_ &M); // multiply with matrix
  Matrix_  operator* (const FLOAT &s);  // multiply with scalar
  Matrix_  operator/ (const Matrix_ &M); // divide elementwise by matrix (or vector)
  Matrix_  operator/ (const FLOAT &s);  // divide by scalar
  Matrix_  operator- ();                // negative matrix
  Matrix_  operator~ ();                // transpose
  FLOAT   l2norm ();                   // euclidean norm (vectors) / frobenius norm (matrices)
  FLOAT   mean ();                     // mean of all elements in matrix

  // complex arithmetic operations
  static Matrix_ cross (const Matrix_ &a, const Matrix_ &b);    // cross product of two vectors
  static Matrix_ inv (const Matrix_ &M);                       // invert matrix M
  bool   inv ();                                             // invert this matrix
  FLOAT  det ();                                             // returns determinant of matrix
  bool   solve (const Matrix_ &M,FLOAT eps=1e-20);            // solve linear system M*x=B, replaces *this and M
  bool   lu(int32_t *idx, FLOAT &d, FLOAT eps=1e-20);        // replace *this by lower upper decomposition
  void   svd(Matrix_ &U,Matrix_ &W,Matrix_ &V);                 // singular value decomposition *this = U*diag(W)*V^T

  // print matrix to stream
  friend std::ostream& operator<< (std::ostream& out,const Matrix_& M);

  // direct data access
  FLOAT   **val;
  int32_t   m,n;

private:

  void allocateMemory (const int32_t m_,const int32_t n_);
  void releaseMemory ();
  inline FLOAT pythag(FLOAT a,FLOAT b);

};

#endif // MATRIX_H
