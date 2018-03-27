#pragma once
#include "LoboFunctions.h"

/*
  Polar decomposition of a general 3x3 matrix

  This code uses the polar decomposition implementation provided as a companion to the book "Graphics Gems IV": 
  Decompose.c 
  Ken Shoemake, 1993 
  Polar Decomposition of 3x3 matrix in 4x4, M = QS.  
  The Graphics Gems IV implementation is available at: 
  http://tog.acm.org/GraphicsGems/

  The above website states that "All code here (on the GraphicsGems website) can be used without restrictions". It also lists the following EULA:
  "EULA: The Graphics Gems code is copyright-protected. In other words, you cannot claim the text of the code as your own and resell it. Using the code is permitted in any program, product, or library, non-commercial or commercial. Giving credit is not required, though is a nice gesture. The code comes as-is, and if there are any flaws or problems with any Gems code, nobody involved with Gems - authors, editors, publishers, or webmasters - are to be held responsible. Basically, don't be a jerk, and remember that anything free comes with no guarantee."

  Jernej Barbic made some adaptions to the polar decomposition code (wrap into a C++ class, some change in input/output format, etc.). 
  He releases his adaptions of the polar decomposition code into the public domain, free of charge. The above EULA still applies, of course.
*/

void polarDecomposition(Matrix3d &M, Matrix3d &Q, Matrix3d &S, double tolerance = 1E-6, int forceRotation = 0)
{
	double det, M_oneNorm, M_infNorm, E_oneNorm;
	int useSVD = 0;

	Matrix3d MK = M.transpose();
	Matrix3d EK;
	M_oneNorm = MK.lpNorm<1>();
	M_infNorm = MK.lpNorm<Eigen::Infinity>();

	do{
		det = MK.determinant();
		if ((det <= 1e-6) && forceRotation)
		{
			useSVD = 1;
			break;
		}

		if (det == 0.0)
		{
			std::cout << "Warning (polarDecomposition) : zero determinant encountered." << std::endl;
			break;
		}

		Matrix3d MadjTk = MK.adjoint().transpose();

		double MadjT_one = (MadjTk).lpNorm<1>();
		double MadjT_inf = MadjTk.lpNorm<Eigen::Infinity>();

		double gamma = sqrt(sqrt((MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm * det * det)));
		double g1 = gamma * 0.5;
		double g2 = 0.5 / (gamma * det);

		EK = MK;
		MK = g1*MK + g2*MadjTk;
		EK -= MK;

		E_oneNorm = EK.lpNorm<1>();
		M_oneNorm = MK.lpNorm<1>();
		M_infNorm = MK.lpNorm<Eigen::Infinity>();

	} while (E_oneNorm > M_oneNorm*tolerance);

	if (useSVD)
	{
		// use the SVD algorithm to compute Q
		Matrix3d Mm(M);
		double modifiedSVD_singularValue_eps = tolerance;
		Matrix3d Um, Vm;
		Matrix3d Lambda;
		int modifiedSVD = 1;
		lobo::computeSVD(Mm, Um, Vm, Lambda, modifiedSVD_singularValue_eps, modifiedSVD);

		Q = Um*Vm.transpose();
	}
	else
	{
		Q = MK.transpose();
	}

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			S.data()[3 * j + i] = 0;
			for (int k = 0; k < 3; k++)
			{
				S.data()[3 * j + i] += MK.data()[3 * k + i] * M.data()[3 * j + k];
			}
		}
	}

	// S must be symmetric; enforce the symmetry
	S.data()[1] = S.data()[3] = 0.5*(S.data()[1] + S.data()[3]);
	S.data()[2] = S.data()[6] = 0.5*(S.data()[2] + S.data()[6]);
	S.data()[5] = S.data()[7] = 0.5*(S.data()[5] + S.data()[7]);
}

