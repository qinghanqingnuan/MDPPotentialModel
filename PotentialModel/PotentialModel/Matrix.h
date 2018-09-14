#pragma once

#include "struct_fun.h"


class Matrix {


private:
	double m_A[N_PARAM][N_PARAM];
	double m_b[N_PARAM];
	int m_Pivot[N_PARAM];
	int m_n;

public:
	Matrix(int n);
	~Matrix();
	void m_SetMatrix(double A[N_PARAM][N_PARAM]);
	void m_GetMatrix(double A[N_PARAM][N_PARAM]);
	void m_SetVector(double* b);
	void m_GetVector(double* b);
	double m_TriangDecomp();
	void m_BackSubstitute();

private:
	void m_fswap(double& a, double& b);

};
