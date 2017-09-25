#ifndef LU_MATRIX_H
#define LU_MATRIX_H
/////////////////////////////////////////////////////////////////
#ifndef PI
#define PI 3.141592653589793
#endif
#ifndef INF
#define INF 10000000000
#endif
#ifndef EPS
#define EPS 0.0000000001
#endif

#include "stdlib.h"  //for function "abs()"
#include "math.h"
//#include <iostream>
using namespace std;

class Lu_Matrix{
public:
	long rowN;
	long colN;
	double *data;

public:
	Lu_Matrix(){rowN=0;colN=0;};
	Lu_Matrix(long r, long c=1);
	Lu_Matrix(const Lu_Matrix&);
	Lu_Matrix& operator=(const Lu_Matrix&);
	~Lu_Matrix(){if(rowN*colN>0) delete[] data;}
	int Reset();//set all values to be zero;
	int Set(long r, long c);
	int Set(long r);
	Lu_Matrix Size() const;//返回2*1矩阵，第一个元素为行数，第二个元素为列数
	int Length() const;    //返回矩阵总的元素数
	int RowNum() const;    //返回行数；
	int ColNum() const;    //返回列数
	double MaxValue() const;
	double MaxValue(long int & MinIndex_row,long int & MinIndex_col) const;
	double MinValue() const;
	double MinValue(long int & MinIndex_row,long int & MinIndex_col) const;
	bool SetArrayRow(double a[], int n);//将数组a 按行赋给矩阵，n为数组长度;
	double & operator()(long i,long j=0);
	double  operator()(long i,long j=0) const;

	Lu_Matrix& operator+=(const Lu_Matrix & m);
	Lu_Matrix& operator-=(const Lu_Matrix & m);
	Lu_Matrix& operator*=(const Lu_Matrix & m);
	Lu_Matrix& operator+=(double d);
	Lu_Matrix& operator-=(double d);
	Lu_Matrix& operator*=(double d);

	Lu_Matrix& operator/=(Lu_Matrix m1);
	Lu_Matrix& operator/=(double d);
	    

	int Display() const;
	int Ut();		//Unit Lu_Matrix
	int Inv();		//Inverse

	Lu_Matrix GetRowVec(long r) const;
	Lu_Matrix GetColVec(long c) const;
	Lu_Matrix Abs() const;
};

Lu_Matrix operator-(const Lu_Matrix & m);			//prefix 

Lu_Matrix operator+(const Lu_Matrix & m1,const Lu_Matrix & m2);
Lu_Matrix operator+(const Lu_Matrix & m, double d);
Lu_Matrix operator+(double d, const Lu_Matrix & m);

Lu_Matrix operator-(const Lu_Matrix & m1,const Lu_Matrix & m2);
Lu_Matrix operator-(const Lu_Matrix & m, double d);
Lu_Matrix operator-(double d, const Lu_Matrix & m);

Lu_Matrix operator*(const Lu_Matrix & m1,const Lu_Matrix & m2);
Lu_Matrix operator*(const Lu_Matrix & m, double d);
Lu_Matrix operator*(double d,const Lu_Matrix & m);

Lu_Matrix operator/(Lu_Matrix m, Lu_Matrix m1);
Lu_Matrix operator/(Lu_Matrix m, double d);

Lu_Matrix Tp(const Lu_Matrix & m);	//Transpose
Lu_Matrix Inv(const Lu_Matrix & m);	//Inverse
Lu_Matrix Inv2by2Matrix(const Lu_Matrix & m);	//Inverse
Lu_Matrix Inv3by3Matrix(const Lu_Matrix & m);

double Det(const Lu_Matrix & m); 

Lu_Matrix GetRowVec(const Lu_Matrix & m, long r);
Lu_Matrix GetColVec(const Lu_Matrix & m, long c);
Lu_Matrix AbsOfMatrix(const Lu_Matrix & m);
double SumOfMatrix(const Lu_Matrix & m);
int MyEigen(const Lu_Matrix & MMatrix, Lu_Matrix &MyEigenValues, 
Lu_Matrix &EigenVectors,double deps=0.000001) ;
//void matrix_error3(){
	//MAPS::ReportInfo("matrix index expend the dimension of matrix!");
//};

//////////////////////////////////////////////////////////////////

#endif
