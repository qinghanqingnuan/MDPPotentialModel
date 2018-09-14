#include "stdafx.h"
#include <math.h>
#include "Matrix.h"


Matrix::Matrix(int n)
{
	m_n = N_PARAM;	
}

Matrix::~Matrix()
{
}

void Matrix::m_SetMatrix(double A[N_PARAM][N_PARAM])
{
	
	for (int i = 0; i < m_n; i ++) {
		for (int j = 0; j < m_n; j ++) {
			m_A[i][j] = A[i][j];
		}
	}
}

void Matrix::m_GetMatrix(double A[N_PARAM][N_PARAM])
{
	for (int i = 0; i < m_n; i ++) {
		for (int j = 0; j < m_n; j ++) {
			A[i][j] = m_A[i][j];
		}
	}
}

void Matrix::m_SetVector(double* b)
{
	for (int i = 0; i < m_n; i ++) {
		m_b[i] = b[i];
	}
}

void Matrix::m_GetVector(double* b)
{
	for (int i = 0; i < m_n; i ++) {
		b[i] = m_b[i];
	}
}

double Matrix::m_TriangDecomp()
{
// 运用部分选主元高斯消去法对实矩阵 A 进行三角分解
// 计算矩阵 A 的条件数 Cond(A) 的近似值

	double ek, t, Anorm, Ynorm, Znorm, Cond;
	short i, j, k, l, m;

	double* work = new double [m_n];
	for (i = 0; i < m_n; i ++) {
		work[i] = 0.;
	}

	m_Pivot[m_n - 1] = 1;

	if (m_n == 1) {
		if (m_A[0][0] != 0.) {
			Cond = 1.;
		}
		else {
			Cond = 1.e+32;
		}
		delete []work;
		return Cond;
	}

//	计算 A 的列向量范数的最大值
	Anorm = 0.;
	for (j = 0; j < m_n; j ++) {
		t = 0.;
		for (i = 0; i < m_n; i ++) {
			t += fabs(m_A[i][j]);
		}
		if (t >= Anorm) Anorm = t;
	}

//	运用部分选主元高斯消去法，将 A 变为上三角矩阵
	for (k = 0; k < m_n - 1; k ++) {
//		选主元
		m = k;
		for (i = k + 1; i < m_n; i ++) {
			if (fabs(m_A[i][k]) > fabs(m_A[m][k])) m = i;
		}
		m_Pivot[k] = m;
		if (m != k) m_Pivot[m_n - 1] = -m_Pivot[m_n - 1];
//		将主元换到对角线上
		t = m_A[m][k];
		m_fswap(m_A[m][k], m_A[k][k]);
		if (t != 0.) {
//			计算倍乘因子
			for (i = k + 1; i < m_n; i ++) {
				m_A[i][k] = -m_A[i][k] / t;
			}
//			对主元所在列之后的其它列施行与选主元消去过程相同的运算
			for (j = k + 1; j < m_n; j ++) {
				t = m_A[m][j];
				m_fswap(m_A[m][j], m_A[k][j]);
				if (t == 0.) continue;
				for (i = k + 1; i < m_n; i ++) {
					m_A[i][j] += m_A[i][k] * t;
				}
			}
		}
	}

//	求解方程组 A'*y=e，其中 A' 为 A 的转置矩阵，e 是分量为±1 的向量
	for (k = 0; k < m_n; k ++) {
		t = 0.;
		if (k > 0) {
			for (i = 0; i < k; i ++) {
				t += m_A[i][k] * work[i];
			}
		}
		if (t < 0.) {
			ek = -1.;
		}
		else {
			ek = 1.;
		}
		if (m_A[k][k] == 0.) {
			Cond = 1.E+32;
			delete []work;
			return Cond;
		}
		work[k] = -(ek + t) / m_A[k][k];
	}

	for (l = 0; l < m_n - 1; l ++) {
		k = m_n - l - 2;
		t = 0.;
		for (i = k + 1; i < m_n; i ++) {
			t += m_A[i][k] * work[k];
		}
		work[k] = t;
		m = m_Pivot[k];
		if (m != k)	{
			m_fswap(work[m], work[k]);
		}
	}

//	计算 ‖y‖
	Ynorm = 0.;
	for (i = 0; i < m_n; i ++) {
		Ynorm += fabs(work[i]);
	}

//	求解方程组 A*z=y
	m_SetVector(work);
	m_BackSubstitute();
	m_GetVector(work);

//	计算 ‖z‖
	Znorm = 0.;
	for (i = 0; i < m_n; i ++) {
		Znorm += fabs(work[i]);
	}

	
//	计算 Cond(A) 的近似值
	Cond = Anorm * Znorm / Ynorm;
	if (Cond < 1.) Cond = 1.;
	delete []work;
	return Cond;
}

void Matrix::m_BackSubstitute()
{
//	求解线性方程组 A*X=b，其中 A 已经高斯消去法变为上三角矩阵

	int i, k, l, m;
	double t;

//	对右端项 b 施行与选主元消去过程相同的运算
	if (m_n > 1) {
		for (k = 0; k < m_n - 1; k ++) {
			m = m_Pivot[k];
			t = m_b[m];
			m_fswap(m_b[m], m_b[k]);
			for (i = k + 1; i < m_n; i ++) {
				m_b[i] += m_A[i][k] * t;
			}
		}

//		回代，求得方程组的解
		for (l = 0; l < m_n - 1; l ++) {
			k = m_n - l - 1;
			m_b[k] /= m_A[k][k];
			t = -m_b[k];
			for (i = 0; i < m_n - l - 1; i ++) {
				m_b[i] += m_A[i][k] * t;
			}
		}
	}
	m_b[0] /= m_A[0][0];
}

void Matrix::m_fswap(double& a, double& b)
{
	double t;
	t = a;
	a = b;
	b = t;
}
