// magneto 1.4 magnetometer/accelerometer calibration code
// from http://sailboatinstruments.blogspot.com/2011/08/improved-magnetometer-calibration.html

#include <math.h>
#include <stdlib.h>

#include "mymathlib_matrix.h"

#include "magneto1_4.h"


void magneto_sample(double x, double y, double z, double *ata, double *norm_sum, double *sample_count)
{
    *sample_count += 1.0;
    *norm_sum += sqrt(x * x + y * y + z * z);

    double D[10] = {
        x * x,
        y * y,
        z * z,
        2.0 * y * z,
        2.0 * x * z,
        2.0 * x * y,
        2.0 * x,
        2.0 * y,
        2.0 * z,
        1.0
    };

    Multiply_Self_Transpose(ata, D, 10, 1);
}

void magneto_current_calibration(float BAinv[4][3], double *ata, double norm_sum, double sample_count)
{
    double* S11 = (double *)malloc(6 * 6 * sizeof(double));
    Get_Submatrix(S11, 6, 6, ata, 10, 0, 0);
    double* S12 = (double *)malloc(6 * 4 * sizeof(double));
    Get_Submatrix(S12, 6, 4, ata, 10, 0, 6);
    double* S12t = (double *)malloc(4 * 6 * sizeof(double));
    Get_Submatrix(S12t, 4, 6, ata, 10, 6, 0);
    double* S22 = (double *)malloc(4 * 4 * sizeof(double));
    Get_Submatrix(S22, 4, 4, ata, 10, 6, 6);

    double hm = norm_sum / sample_count;

    // this is where we'd deallocate ata or the entire calibration class
    // if we decided to compute the calibration destructively

    Choleski_LU_Decomposition(S22, 4);
    Choleski_LU_Inverse(S22, 4);

    // Calculate S22a = S22 * S12t   4*6 = 4x4 * 4x6   C = AB
    double* S22a = (double *)malloc(4 * 6 * sizeof(double));
    Multiply_Matrices(S22a, S22, 4, 4, S12t, 6);
    free(S22);
    free(S12t);

    // Then calculate S22b = S12 * S22a      ( 6x6 = 6x4 * 4x6)
    double* S22b = (double *)malloc(6 * 6 * sizeof(double));
    Multiply_Matrices(S22b, S12, 6, 4, S22a, 6);
    free(S12);

    // Calculate SS = S11 - S22b
    double* SS = (double *)malloc(6 * 6 * sizeof(double));
    for (int i = 0; i < 36; i++)
        SS[i] = S11[i] - S22b[i];
    free(S11);
    free(S22b);

    // Create pre-inverted constraint matrix C
    double* C = (double *)malloc(6 * 6 * sizeof(double));
    C[0] = 0.0;
    C[1] = 0.5;
    C[2] = 0.5;
    C[3] = 0.0;
    C[4] = 0.0;
    C[5] = 0.0;
    C[6] = 0.5;
    C[7] = 0.0;
    C[8] = 0.5;
    C[9] = 0.0;
    C[10] = 0.0;
    C[11] = 0.0;
    C[12] = 0.5;
    C[13] = 0.5;
    C[14] = 0.0;
    C[15] = 0.0;
    C[16] = 0.0;
    C[17] = 0.0;
    C[18] = 0.0;
    C[19] = 0.0;
    C[20] = 0.0;
    C[21] = -0.25;
    C[22] = 0.0;
    C[23] = 0.0;
    C[24] = 0.0;
    C[25] = 0.0;
    C[26] = 0.0;
    C[27] = 0.0;
    C[28] = -0.25;
    C[29] = 0.0;
    C[30] = 0.0;
    C[31] = 0.0;
    C[32] = 0.0;
    C[33] = 0.0;
    C[34] = 0.0;
    C[35] = -0.25;
    double* E = (double *)malloc(6 * 6 * sizeof(double));
    Multiply_Matrices(E, C, 6, 6, SS, 6);
    free(C);
    free(SS);

    double* SSS = (double *)malloc(6 * 6 * sizeof(double));
    Hessenberg_Form_Elementary(E, SSS, 6);

    int index = 0;
    {
        double* eigen_real = (double *)malloc(6 * sizeof(double));
        double* eigen_imag = (double *)malloc(6 * sizeof(double));

        QR_Hessenberg_Matrix(E, SSS, eigen_real, eigen_imag, 6, 100);
        free(E);

        double maxval = eigen_real[0];
        for (int i = 1; i < 6; i++)
        {
            if (eigen_real[i] > maxval)
            {
                maxval = eigen_real[i];
                index = i;
            }
        }
        free(eigen_real);
        free(eigen_imag);
    }

    double* v1 = (double *)malloc(6 * sizeof(double));
    v1[0] = SSS[index];
    v1[1] = SSS[index + 6];
    v1[2] = SSS[index + 12];
    v1[3] = SSS[index + 18];
    v1[4] = SSS[index + 24];
    v1[5] = SSS[index + 30];
    free(SSS);

    // normalize v1
    {
        double norm = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] + v1[3] * v1[3] + v1[4] * v1[4] + v1[5] * v1[5]);
        v1[0] /= norm;
        v1[1] /= norm;
        v1[2] /= norm;
        v1[3] /= norm;
        v1[4] /= norm;
        v1[5] /= norm;
    }

    if (v1[0] < 0.0)
    {
        v1[0] = -v1[0];
        v1[1] = -v1[1];
        v1[2] = -v1[2];
        v1[3] = -v1[3];
        v1[4] = -v1[4];
        v1[5] = -v1[5];
    }

    // Calculate v2 = S22a * v1      ( 4x1 = 4x6 * 6x1)
    double* v2 = (double *)malloc(4 * sizeof(double));
    Multiply_Matrices(v2, S22a, 4, 6, v1, 1);
    free(S22a);

    double* U = (double *)malloc(3 * sizeof(double));
    double* Q = (double *)malloc(3 * 3 * sizeof(double));
    double J;
    {
        double* v = (double *)malloc(10 * sizeof(double));
        v[0] = v1[0];
        v[1] = v1[1];
        v[2] = v1[2];
        v[3] = v1[3];
        v[4] = v1[4];
        v[5] = v1[5];
        free(v1);
        v[6] = -v2[0];
        v[7] = -v2[1];
        v[8] = -v2[2];
        v[9] = -v2[3];
        free(v2);

        Q[0] = v[0];
        Q[1] = v[5];
        Q[2] = v[4];
        Q[3] = v[5];
        Q[4] = v[1];
        Q[5] = v[3];
        Q[6] = v[4];
        Q[7] = v[3];
        Q[8] = v[2];

        U[0] = v[6];
        U[1] = v[7];
        U[2] = v[8];

        J = v[9];
        free(v);
    }

    double* B = (double *)malloc(3 * sizeof(double));
    {
        double* Q_1 = (double *)malloc(3 * 3 * sizeof(double));
        for (int i = 0; i < 9; i++)
            Q_1[i] = Q[i];
        Choleski_LU_Decomposition(Q_1, 3);
        Choleski_LU_Inverse(Q_1, 3);

        // Calculate B = Q-1 * U   ( 3x1 = 3x3 * 3x1)
        Multiply_Matrices(B, Q_1, 3, 3, U, 1);
        free(U);
        free(Q_1);
        B[0] = -B[0]; // x-axis combined bias
        B[1] = -B[1]; // y-axis combined bias
        B[2] = -B[2]; // z-axis combined bias
    }

    // First calculate QB = Q * B   ( 3x1 = 3x3 * 3x1)
    double btqb;
    {
        double* QB = (double *)malloc(3 * sizeof(double));
        Multiply_Matrices(QB, Q, 3, 3, B, 1);

        // Then calculate btqb = BT * QB    ( 1x1 = 1x3 * 3x1)
        Multiply_Matrices(&btqb, B, 1, 3, QB, 1);
        free(QB);
    }

    // Calculate SQ, the square root of matrix Q
    double* SSSS = (double *)malloc(3 * 3 * sizeof(double));
    Hessenberg_Form_Elementary(Q, SSSS, 3);

    double* Dz = (double *)malloc(3 * 3 * sizeof(double));
    for (int i = 0; i < 9; i++)
    {
        Dz[i] = 0;
    }
    {
        double* eigen_real3 = (double *)malloc(3 * sizeof(double));
        double* eigen_imag3 = (double *)malloc(3 * sizeof(double));
        QR_Hessenberg_Matrix(Q, SSSS, eigen_real3, eigen_imag3, 3, 100);
        free(Q);

        Dz[0] = sqrt(eigen_real3[0]);
        Dz[4] = sqrt(eigen_real3[1]);
        Dz[8] = sqrt(eigen_real3[2]);
        free(eigen_real3);
        free(eigen_imag3);
    }

    {
        // normalize eigenvectors
        double norm = sqrt(SSSS[0] * SSSS[0] + SSSS[3] * SSSS[3] + SSSS[6] * SSSS[6]);
        SSSS[0] /= norm;
        SSSS[3] /= norm;
        SSSS[6] /= norm;
        norm = sqrt(SSSS[1] * SSSS[1] + SSSS[4] * SSSS[4] + SSSS[7] * SSSS[7]);
        SSSS[1] /= norm;
        SSSS[4] /= norm;
        SSSS[7] /= norm;
        norm = sqrt(SSSS[2] * SSSS[2] + SSSS[5] * SSSS[5] + SSSS[8] * SSSS[8]);
        SSSS[2] /= norm;
        SSSS[5] /= norm;
        SSSS[8] /= norm;
    }

    double* SQ = (double *)malloc(3 * 3 * sizeof(double));
    {
        double* vdz = (double *)malloc(3 * 3 * sizeof(double));
        ;
        Multiply_Matrices(vdz, SSSS, 3, 3, Dz, 3);
        free(Dz);
        Transpose_Square_Matrix(SSSS, 3);
        Multiply_Matrices(SQ, vdz, 3, 3, SSSS, 3);
        free(SSSS);
        free(vdz);
    }

    double* A_1 = (double *)malloc(3 * 3 * sizeof(double));
    // Calculate hmb = sqrt(btqb - J).
    double hmb = sqrt(btqb - J);

    for (int i = 0; i < 9; i++)
        A_1[i] = SQ[i] * hm / hmb;
    free(SQ);

    for (int i = 0; i < 3; i++)
        BAinv[0][i] = B[i];
    free(B);

    for (int i = 0; i < 3; i++)
    {
        BAinv[i + 1][0] = A_1[i * 3];
        BAinv[i + 1][1] = A_1[i * 3 + 1];
        BAinv[i + 1][2] = A_1[i * 3 + 2];
    }
    free(A_1);
}
