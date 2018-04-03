#ifndef __JACOBI_SOLVER_H__
#define __JACOBI_SOLVER_H__

#include "sparse_matrix.h"
#include "Vector.h"

/*!
* \class JacobiSolver
* source: https://github.com/ChaliZhg/cfdlab
* \date August 2016
*/

template <class T>
class JacobiSolver
{
   public:
      JacobiSolver (unsigned int max_iter,T tol);
      ~JacobiSolver () {};
	  unsigned int solve(const cfdlab::SparseMatrix<T>& A,
								cfdlab::Vector<T>& x,
						const    cfdlab::Vector<T>& f) const;

   private:
      unsigned int max_iter;
      T            tol;
};

#endif
