#ifndef RMATRIX_H
#define RMATRIX_H

#include<vector>
#include<iomanip>
class RVector;
using namespace std;
#define MATRIX_SIZE 70
class RMatrix
{
public:
    RMatrix();
    RMatrix(size_t m);
    RMatrix(size_t m, size_t n);
    RMatrix(double m[9][9]);
    RMatrix(double a[4][4], size_t m);
    RMatrix(double *eerot, double *eetrans);
    RMatrix(const RMatrix rot, double *eetrans);

    ~RMatrix(void);

    RMatrix(const RMatrix &other);
    RMatrix operator=(const RMatrix &other);

    friend RMatrix operator+(const RMatrix &A,const RMatrix &B);
//    friend RMatrix operator*(const RMatrix &A,const RMatrix &B);
    friend RMatrix operator-(const RMatrix &A,const RMatrix &B);
    RVector operator*(const RVector &other);

    RMatrix operator*(const RMatrix &other);
//    RMatrix operator*(const RMatrix &A, const RMatrix &B);


//    static RMatrix RTranspose(RMatrix A);
    static RMatrix RTranspose(const RMatrix other);
    static bool RMatrixInv(RMatrix A, RMatrix &B, int size);
    static bool RMatrixInv(RMatrix A, RMatrix &B);

    static RMatrix subRMatrix(RMatrix A, size_t m0, size_t n0, size_t m1, size_t n1);
    static RVector subRVector(RMatrix &A,size_t m0,size_t m1,size_t n,string type);
    static void catRMatrix(RMatrix &A, size_t m0, size_t n0, size_t m1, size_t n1, RMatrix &B);
    static void catRMatrix(RMatrix &A, size_t m0, size_t m1, size_t n0, RVector &b);
    static void replaceRMatrix(RMatrix &A,size_t m0,string type,RVector &B);

    static RMatrix RIdentity(size_t n);
    static void clearRMatrix(RMatrix &A);

    static void svdSim(RMatrix &A,RMatrix &U,RMatrix &S,RMatrix &V);
    static void qrFullRMatrix(RMatrix &A,RMatrix &Q);
    static RMatrix triuRMatrix(RMatrix &A);
    static double normRMatrix(RMatrix &A,size_t n);

    static RMatrix Quaternion2RotMatrix(RVector &a);
    static void replaceOriOfTMatrix(RMatrix &A, RMatrix B);
    static void replaceDisOfTMatrix(RMatrix &A, RVector &b);
    static RMatrix RPY2TR(RVector rpy);

    static RMatrix RotZ(double t);
    static RMatrix RotY(double t);
    static RMatrix RotX(double t);


        double value[MATRIX_SIZE][MATRIX_SIZE];
//    vector<vector<double> > value;
    size_t iRow;
    size_t iCol;
    bool IsAmpty;
};

#endif // RMATRIX_H
