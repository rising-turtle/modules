#ifndef _CEIGENMETRIX
#define _CEIGENMETRIX
#include <cmath>
#include <iostream>
using namespace std;

class CEigenM44{
public:
	static const int dim =4;
	static const int one_dim = 16;
	CEigenM44(){ Initialize();}
	CEigenM44(double N[],int n=16){SetValues(N,n);}
	CEigenM44(double N[dim][dim]){SetValues(N);}
	~CEigenM44(){}
	void SetValue(int i,int j, double val){_d[i][j] = val;
	_od[i*dim+j] = val;}
	
	void MetrixMultiple44(double u[dim]);
	//void MetrixNormalization(double u[dim],int &index,double &m);
	
	void EigenVectorMax(double vecs[dim],double& val);
	
private:
	double _d[dim][dim];
	double _od[one_dim];
	int eejcb(double a[],int n,double v[],double eps,int jt);
	void Initialize(){static double t[dim][dim];SetValues(t);}
	void SetValues(double const N[dim][dim]);
	void SetValues(double const N[],int n);
	explicit CEigenM44(CEigenM44&);
	CEigenM44& operator = (CEigenM44&);
};


#endif