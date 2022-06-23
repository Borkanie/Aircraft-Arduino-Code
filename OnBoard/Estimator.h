#include <Core>
#include <LU>
#pragma once
using namespace Eigen;
namespace Estimator
{
    void subMatrix(const float mat[9][9], float temp[9][9], int p, int q, int n);
    float determinantOfMatrix(const float matrix[9][9], float n);
   
    class ExtendedSystem
    {
    private:
        const Matrix2i I12=Identity(12, 12);
        // Our estimator actually estimates deviance from equilibrium conditions.
        static float DeltaInputs[5];
        static float DeltaStates[12];
        static float Ppred[12][12];
        // The values of equilibrium for inputs.
        const float Inputs[5] = {5.5, 0, 0, 0, 0};
        // The values of equilibrium for states.
        /*The states are:
        {
        Velocity in X axis along body reference frame.
        Velocity in Y axis along body reference frame.
        Velocity in Z axis along body reference frame.
        Angular velocity along X axis on body reference frame.
        Angular velocity along Y axis on body reference frame.
        Angular velocity along Z axis on body reference frame.
        Euler angle X.
        Euler angle Y.
        Euler angle Z.
        Wind velocity in X axis along body reference frame.
        Wind velocity in Y axis along body reference frame.
        Wind velocity in Z axis along body reference frame.
        }
        */
        // In order to stay in equilibrium the sum of velocity vectors must be null.
        // All other states are irrelevant regarding translational movement in this model.
        const float States[9] = {11.8, 0, 1.8, 0, 0, 0, 0, 0.1, 0};
        static float P[12][12];
        const float Q[12][12] = {
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}};
        const float R[9][9] = {
            {3, 3, 3, 3, 3, 3, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3, 3},
            {3, 3, 3, 3, 3, 3, 3, 3, 3}};
        // Extended liniarized system matrix A.
        const float AE[12][12] = {
            {0.8226, -0.0162, -0.1797, 0, -0.2800, 0.1000, 0, -1.0272, 0.1336, 0.1774, 0.0162, 0.1797},
            {-0.0095, 0.8647, -0.0044, 0.2800, 0, -1.2800, -0.9761, 0, -1.1879, 0.0095, 0.1353, 0.0044},
            {-0.1376, -0.0232, 0.2065, -0.1000, 1.2800, 0, 0, 1.2969, -0.0134, 0.1376, 0.0232, 0.7935},
            {-0.0000, -0.0003, -0.0000, 1.0000, 0, 0, 0, 0, 0, 0.0000, 0.0270, 0.0000},
            {0, 0, 0, 0, 1.0000, 0, 0, 0, 0, 0, 0, 0},
            {-0.0001, -0.0019, -0.0001, 0, 0, 1.0000, 0, 0, 0, 0.0001, 0.0019, 0.0001},
            {0, 0, 0, 0.1000, 0, 0.0100, 1.0000, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0.1000, 0, 0, 1.0000, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0.1005, 0, 0, 1.0000, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000}};
        // Extended liniarized system matrix B.
        const float BE[12][5] = {
            {0.1587, -0.0643, 0, -0.0185, -0.0185},
            {0, 0, -0.1353, 0, 0},
            {0, -0.6408, 0, -0.1842 - 0.1842},
            {0, 0, -0.0061, 0.0414, -0.0414},
            {0, -0.2434, 0, 0, 0},
            {0, 0, -0.0430, 0.0018 - 0.0018},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0}};
        // Extended liniarized system matrix C.
        const float CE[9][12] = {
            {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}};
        static float KalmanGain[12][9];
        // Linear fullstate feedback controller for airspeed control.
        const float FSFController[5][12]{
            {0.9884, -0.0629, -0.2409, 0.0364, -1.4949, 0.7359, 0.2943, -3.4698, 0.7385, -0.9884, 0.0629, 0.2409},
            {0.0687, -0.0010, -0.0350, 0.0236, -1.2925, 0.1366, 0.0181, -1.4199, 0.1369, -0.0687, 0.0010, 0.0350},
            {-0.0008, 0.0501, 0.0001, -0.4123, 0.0053, -3.1612, -1.0159, 0.0104, -1.4229, 0.0008, -0.0501, -0.0001},
            {0.0065, -0.0297, -0.0078, 1.4937, -0.0358, 2.2694, 1.2559, -0.0489, 1.1442, -0.0065, 0.0297, 0.0078},
            {0.0058, 0.0323, -0.0076, -1.4853, -0.0291, -2.2804, -1.2620, -0.0377, -1.1510, -0.0058, -0.0323, 0.0076}};
        // Sampling time.
        const float Ts = 0.1;
        void Predict();
        void CalculatePpred();
        void CalculateNewGain();
        void Estimate(float readings[9]);
        void Innovate();
        static float Xpred[12];
        void MatrixMultiply(const float fistMatrix[12][12], const float secondMatrix[12], float (&resultMatrix)[12]);
        void MatrixMultiply(const float fistMatrix[9][12], const float secondMatrix[12][12], float (&resultMatrix)[9][12]);
        void MatrixMultiply(const float fistMatrix[12][12], const float secondMatrix[12][9], float (&resultMatrix)[9][12]);
        void MatrixMultiply(const float fistMatrix[9][12], const float secondMatrix[12][9], float (&resultMatrix)[9][9]);
        void MatrixMultiply(const float fistMatrix[12][9], const float secondMatrix[9][12], float (&resultMatrix)[12][12]);
        void MatrixMultiply(const float fistMatrix[12][9], const float secondMatrix[9][9], float (&resultMatrix)[12][9]);
        void MatrixMultiply(const float fistMatrix[12][5], const float secondMatrix[5], float (&resultMatrix)[12]);
        void MatrixMultiply(const float fistMatrix[12][9], const float secondMatrix[9], float (&resultMatrix)[12]);
        void MatrixMultiply(const float fistMatrix[9][12], const float secondMatrix[12], float (&resultMatrix)[9]);
        void MatrixMultiply(const float fistMatrix[12][12], const float secondMatrix[12][12], float (&resultMatrix)[12][12]);
        void MatrixMultiply(const float fistMatrix[12][12], const float secondMatrix[12][9], float (&resultMatrix)[12][9]);
        void MatrixAdd(const float fistMatrix[12][12], const float secondMatrix[12][12], float (&resultMatrix)[12][12]);
        void MatrixAdd(const float fistMatrix[9][9], const float secondMatrix[9][9], float (&resultMatrix)[9][9]);
        void MatrixAdd(const float fistMatrix[12], const float secondMatrix[12], float (&resultMatrix)[12]);
        void Transpose(const float sourceMatrix[12][12], float (&resultMatrix)[12][12]);
        void Inverse(const float sourceMatrix[12][12], float (&resultMatrix)[12][12]);
        void Inverse(const float sourceMatrix[9][9], float (&resultMatrix)[9][9]);
        void Transpose(const float sourceMatrix[9][12], float (&resultMatrix)[12][9]);
        void Transpose(const float sourceMatrix[9][9], float (&resultMatrix)[9][9]);
        void Transpose(const float sourceMatrix[12][9], float (&resultMatrix)[9][12]);
    public:
     
        void DoKalmanAlgorithm(float readings[9]);
    };
}