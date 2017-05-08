#include "CEigenMetrix.h"

static const double eps = 1e-15;
static const int dim=4;

void CEigenM44::SetValues(double const N[dim][dim])
{
	for(int i=0;i<dim;i++)
		for(int j=0;j<dim;j++)
		{
			this->_d[i][j] = N[i][j];
			this->_od[i*dim +j] = N[i][j];
		}
}
void CEigenM44::SetValues(double const N[],int n)
{
	if(n<16) cerr<<"error in SetValues(N,n)"<<endl;
	for(int i=0;i<n;i++)
	{
		_od[i] = N[i];
	}
	for(int i=0;i<dim;i++)
		for(int j=0;j<dim;j++)
			_d[i][j] = _od[i*dim + j];
}
void CEigenM44::MetrixMultiple44(double u[dim]){
	double tmp[dim] ={0,0,0};
	    for(int i=0;i<dim;i++)
			for(int j=0;j<dim;j++)
					tmp[i] += _d[i][j]*u[j];
		for(int i=0;i<dim;i++)
			u[i] = tmp[i];
		return ;
}


void CEigenM44::EigenVectorMax(double vecs[dim],double& val){
	
		double u[CEigenM44::one_dim];
		double a[CEigenM44::one_dim];
		for(int i=0;i<CEigenM44::one_dim;i++)
			a[i] = _od[i];
		eejcb(a,dim,u,eps,50);
		double max = -1;
		int index;
		for(int i=0;i<dim;i++){
			if(max < a[i*4+i])
			{
				max = a[i*4+i];
				index = i;
			}
		}
		val = a[index*dim + index];
		for(int i=0;i<dim;i++)
			vecs[i] = u[i* dim +index];
		cout<<"Max eigen_val: "<<val<<" Mac eigen_vector: {"<<vecs[0]<<" , "<<vecs[1]<<" , "<<vecs[2]<<", "<<vecs[3]<<" }"<<endl;
}

//求实对称矩阵的特征值及特征向量的雅格比法
//利用雅格比(Jacobi)方法求实对称矩阵的全部特征值及特征向量
//返回值小于0表示超过迭代jt次仍未达到精度要求
//返回值大于0表示正常返回
//a-长度为n*n的数组，存放实对称矩阵，返回时对角线存放n个特征值
//n-矩阵的阶数
//u-长度为n*n的数组，返回特征向量(按列存储)
//eps-控制精度要求
//jt-整型变量，控制最大迭代次数
int CEigenM44::eejcb(double a[],int n,double v[],double eps,int jt)
{ 
int i,j,p,q,u,w,t,s,l;
     double fm,cn,sn,omega,x,y,d;
     l=1;
     for (i=0; i<=n-1; i++)
{ 
   v[i*n+i]=1.0;
         for (j=0; j<=n-1; j++)
   {
   if (i!=j) 
   {
     v[i*n+j]=0.0;
   }
   }
}
     while (1==1)
{ 
   fm=0.0;
         for (i=0; i<=n-1; i++)
   {
   for (j=0; j<=n-1; j++)
   { 
     d=fabs(a[i*n+j]);
     if ((i!=j)&&(d>fm))
     { 
     fm=d; 
     p=i; 
     q=j;
     }
   }
   }
         if (fm<eps)  
   {
   return(1);
   }
         if (l>jt)  
   {
   return(-1);
   }
         l=l+1;
         u=p*n+q; 
   w=p*n+p; 
   t=q*n+p; 
   s=q*n+q;
         x=-a[u];
   y=(a[s]-a[w])/2.0;
         omega=x/sqrt(x*x+y*y);
         if (y<0.0)
   {
   omega=-omega;
   }
         sn=1.0+sqrt(1.0-omega*omega);
         sn=omega/sqrt(2.0*sn);
         cn=sqrt(1.0-sn*sn);
         fm=a[w];
         a[w]=fm*cn*cn+a[s]*sn*sn+a[u]*omega;
         a[s]=fm*sn*sn+a[s]*cn*cn-a[u]*omega;
         a[u]=0.0;
   a[t]=0.0;
         for (j=0; j<=n-1; j++)
   {
   if ((j!=p)&&(j!=q))
   { 
     u=p*n+j;
     w=q*n+j;
     fm=a[u];
     a[u]=fm*cn+a[w]*sn;
     a[w]=-fm*sn+a[w]*cn;
   }
   }
         for (i=0; i<=n-1; i++)
   {
   if ((i!=p)&&(i!=q))
             { 
     u=i*n+p; 
     w=i*n+q;
     fm=a[u];
     a[u]=fm*cn+a[w]*sn;
     a[w]=-fm*sn+a[w]*cn;
             }
   }
         for (i=0; i<=n-1; i++)
   { 
   u=i*n+p; 
   w=i*n+q;
             fm=v[u];
             v[u]=fm*cn+v[w]*sn;
             v[w]=-fm*sn+v[w]*cn;
   }
}
return(1);
}