// ------------------------------
// Written by Suleyman TARSUSLU
// ------------------------------
#include "homography.hpp"

extern "C" {

        extern void dgesvd_(char* jobu, char* jobvt, int* m, int* n,
                            double* A, int* lda,
                            double* S, double* U, int* ldu, double* VT, int* ldvt,
                            double* work, int* lwork,
                            int* info);
}

namespace ceng391 {

static void svd(char jobU, char jobVt, int m, int n,
                double* A, int ldA, double* S, double* U, int ldU,
                double* Vt, int ldVt)
{
        double work_tmp = 0;
        int lwork = -1;

        int info;
        dgesvd_(&jobU, &jobVt, &m, &n, nullptr, &ldA, nullptr, nullptr, &ldU,
                nullptr, &ldVt, &work_tmp, &lwork, &info);

        lwork = (int)work_tmp;
        double* work  = new double[lwork];

        dgesvd_(&jobU, &jobVt, &m, &n, A, &ldA, S, U, &ldU, Vt, &ldVt,
                work, &lwork, &info);

        delete [] work;
}

static void svd_svt(double *S, double *Vt, int ldVt, int m, int n,
                    double *A, int ldA)
{
        char jobU  = 'N';
        char jobVt = 'A';
        svd(jobU, jobVt, m, n, A, ldA, S, nullptr, m, Vt, ldVt);
}

void fit_homography4(const double *matches, double *h)
{
        double *A = new double[8*9];
        for (int i = 0; i < 4; ++i) {
                A[0  + i * 2] = 0.0;
                A[8  + i * 2] = 0.0;
                A[16 + i * 2] = 0.0;
                A[24 + i * 2] = -matches[0 + i*4];
                A[32 + i * 2] = -matches[1 + i*4];
                A[40 + i * 2] = -1.0;
                A[48 + i * 2] = matches[3 + i*4] * matches[0 + i*4];
                A[56 + i * 2] = matches[3 + i*4] * matches[1 + i*4];
                A[64 + i * 2] = matches[3 + i*4];

                A[1  + i * 2] = matches[0 + i*4];
                A[9  + i * 2] = matches[1 + i*4];
                A[17 + i * 2] = 1.0;
                A[25 + i * 2] = 0.0;
                A[33 + i * 2] = 0.0;
                A[41 + i * 2] = 0.0;
                A[49 + i * 2] = -matches[2 + i*4] * matches[0 + i*4];
                A[57 + i * 2] = -matches[2 + i*4] * matches[1 + i*4];
                A[65 + i * 2] = -matches[2 + i*4];
        }

        double S[9];
        double Vt[9*9];
        svd_svt(&S[0], &Vt[0], 9, 8, 9, A, 8);

        h[0] = Vt[8];
        h[3] = Vt[17];
        h[6] = Vt[26];

        h[1] = Vt[35];
        h[4] = Vt[44];
        h[7] = Vt[53];

        h[2] = Vt[62];
        h[5] = Vt[71];
        h[8] = Vt[80];

        delete [] A;
}

}
