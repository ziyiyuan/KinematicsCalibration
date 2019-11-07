#include "utility/include/rmatrix.h"
#include "utility/include/rvector.h"
#include <iostream>

using namespace std;
// maxinput A 6*6, A = Q*R;
bool maqr(const RMatrix A, RMatrix& Q, RMatrix& R)
{
    size_t m = A.iRow;
    size_t n = A.iCol;

    double a[36] = {0}, q[36] = {0};
    int i,j,k,l,nn,p,jj;
    double u,alpha,w,t;

    for(int i = 0; i < m; i++)
    {
        for(int j = 0; j < n; j++)
        {
            a[i*n+j] = A.value[i][j];
        }
    }

    if (m<n)
    {
        std::cout<<"fail"<<std::endl;
        return false;
    }
    else
    {
        for (i=0; i<=m-1; i++)
            for (j=0; j<=m-1; j++)
            {
                l=i*m+j; q[l]=0.0;
                if (i==j) q[l]=1.0;
            }
        nn=n;
        if (m==n) nn=m-1;
        for (k=0; k<=nn-1; k++)
        {
            u=0.0; l=k*n+k;
            for (i=k; i<=m-1; i++)
            {
                w=fabs(a[i*n+k]);
                if (w>u) u=w;
            }
            alpha=0.0;
            for (i=k; i<=m-1; i++)
            {
                t=a[i*n+k]/u;
                alpha=alpha+t*t;
            }
            if (a[l]>0.0) u=-u;
            alpha=u*sqrt(alpha);
            if (fabs(alpha)+1.0==1.0)
            {
                std::cout<<"fail"<<std::endl;
                return false;
            }
            u=sqrt(2.0*alpha*(alpha-a[l]));
            if ((u+1.0)!=1.0)
            {
                a[l]=(a[l]-alpha)/u;
                for (i=k+1; i<=m-1; i++)
                {
                    p=i*n+k; a[p]=a[p]/u;
                }
                for (j=0; j<=m-1; j++)
                {
                    t=0.0;
                    for (jj=k; jj<=m-1; jj++)
                        t=t+a[jj*n+k]*q[jj*m+j];
                    for (i=k; i<=m-1; i++)
                    {
                        p=i*m+j; q[p]=q[p]-2.0*t*a[i*n+k];
                    }
                }
                for (j=k+1; j<=n-1; j++)
                {
                    t=0.0;
                    for (jj=k; jj<=m-1; jj++)
                        t=t+a[jj*n+k]*a[jj*n+j];
                    for (i=k; i<=m-1; i++)
                    {
                        p=i*n+j; a[p]=a[p]-2.0*t*a[i*n+k];
                    }
                }
                a[l]=alpha;
                for (i=k+1; i<=m-1; i++)
                    a[i*n+k]=0.0;
            }
        }
        for (i=0; i<=m-2; i++)
            for (j=i+1; j<=m-1;j++)
            {
                p=i*m+j; l=j*m+i;
                t=q[p]; q[p]=q[l]; q[l]=t;
            }

        for(int i = 0; i < m; i++)
        {
            for(int j = 0; j < m; j++)
            {
                Q.value[i][j] = q[i*m+j];
            }
        }

        for(int i = 0; i < m; i++)
        {
            for(int j = 0; j < n; j++)
            {
                R.value[i][j] = a[i*n+j];
            }
        }
        return true;
    }
}
// VERIFY S;but there are same difference with U,V
void svdSim(RMatrix A,RMatrix &U,RMatrix &S,RMatrix &V)
{
    size_t m = A.iRow;
    size_t n = A.iCol;

    RMatrix U_temp(m), V_temp(n);
    S = RMatrix::RTranspose(A);

    double tol = 1e-13;			//精度控制
    size_t loopmax = 300;			//迭代次数l
    size_t loopcount = 0;			//计数器

    double Err =  10000000;  //1.7977e+308
    while (Err > tol && loopcount < loopmax)
    {
        // log10([Err tol loopcount loopmax]); pause

        RMatrix S_t(m,n);
        S_t = RMatrix::RTranspose(S);

        RMatrix Q(m,m), R(m,n), R_t(n,m);
        RMatrix Q1(n,n), R1(n,m);
        bool flag = maqr(S_t, Q, R);
        U_temp = U_temp * Q;

        R_t = RMatrix::RTranspose(R);

        flag = maqr(R_t, Q1, R1);

        V_temp =V_temp * Q1;

        S = R1;
        /*RMatrix E(S.iRow,S.iCol);*/
        RMatrix E = RMatrix::triuRMatrix(S);
        double e = RMatrix::normRMatrix(E,1);
        double f = 0;
        for (size_t i = 0;i < n; i++) f += S.value[i][i] * S.value[i][i];
        f = sqrt(f);
        if (f == 0) f = 1;
        Err = e / f;
        loopcount = loopcount + 1;
    }
    U = U_temp;
    V = V_temp;

    //调整输出U S 矩阵
    RVector SS = RVector::diagRVector(S);			//获得S矩阵的对角元素
    RMatrix::clearRMatrix(S);
    double ssn = 0;
    for(size_t i=0;i<SS.size;i++)
    {
        ssn = SS.value[i];
        S.value[i][i] = fabs(ssn);
        if (ssn < 0)
        {
            for(size_t j=0;j<U.iRow;j++)
                U.value[j][i] = -U.value[j][i];
        }
    }
}



