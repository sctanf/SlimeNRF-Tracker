#ifndef MAGNETO1_4_H
#define MAGNETO1_4_H
void magneto_sample(double x, double y, double z, double* ata, double* norm_sum, double* sample_count);
void magneto_current_calibration(float BAinv[4][3], double* ata, double norm_sum, double sample_count);
#endif