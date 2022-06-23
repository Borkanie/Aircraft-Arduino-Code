#include "Arduino.h"
#include "Estimator.h"

namespace Estimator
{
    void ExtendedSystem::DoKalmanAlgorithm(float readings[9])
    {
        this->Predict();
        this->CalculatePpred();
        this->CalculateNewGain();
        this->Estimate(readings);
        this->Innovate();
    }
    void ExtendedSystem::CalculatePpred()
    {
        float resultM[12][12];
        MatrixMultiply(this->AE, this->P, resultM);
        float AET[12][12];
        Transpose(this->AE, AET);
        float resultM1[12][12];
        MatrixMultiply(resultM, AET, resultM1);
        MatrixAdd(resultM1, this->Q, this->Ppred);
    }
    void ExtendedSystem::CalculateNewGain()
    {
        float resultM1[9][12];
        MatrixMultiply(this->CE, this->Ppred, resultM1);
        float CET[12][9];
        Transpose(this->CE, CET);
        float resultM2[9][9];
        MatrixMultiply(resultM1, CET, resultM2);
        float resultM3[9][9];
        MatrixAdd(resultM2, this->R, resultM3);
        Inverse(resultM3, resultM2);
        float resultM4[12][9];
        MatrixMultiply(this->Ppred,CET,resultM4);
        MatrixMultiply(resultM4,resultM2,this->KalmanGain);
    }
    void ExtendedSystem::Estimate(float readings[9])
    {
        float result1[9];
        MatrixMultiply(this->CE,this->Xpred,result1);
        for(int i=0;i<9;i++)
        {
            result1[i]-=(readings[i]-this->States[i]);
        }
        float innovation[12];
        MatrixMultiply(this->KalmanGain,result1,innovation);
        MatrixAdd(this->Xpred,innovation,this->DeltaStates);
    } 
    void ExtendedSystem::Innovate()
    {
        float result1[12][9];
        //K*R
        MatrixMultiply(this->KalmanGain,this->R,result1);
        float kalmanTranspose[9][12];
        //K'
        Transpose(this->KalmanGain,kalmanTranspose);
        float freeInovationTerm[12][12];
        //K*R*K'
        MatrixMultiply(result1,kalmanTranspose,freeInovationTerm);

        float IminusKalmanC[12][12];
        float KalmanC[12][12];
        //KC
        MatrixMultiply(this->KalmanGain,this->CE,KalmanC);
        for(int i=0;i<12;i++)
        {
            for(int j=0;j<12;j++)
            {
                KalmanC[i][j]=-KalmanC[i][j];
            }
        }
        //(I-KC)
        MatrixAdd(this->I12,KalmanC,IminusKalmanC);
        float IminusKalmanCTranspose[12][12];
        //(I-KC)'
        Transpose(IminusKalmanC,IminusKalmanCTranspose);
        //(I-KC)*Ppred
        MatrixMultiply(IminusKalmanC,this->Ppred,KalmanC);
        //(I-KC)*Ppred*(I-KC)'
        MatrixMultiply(KalmanC,IminusKalmanCTranspose,IminusKalmanC);
         //(I-KC)*Ppred*(I-KC)'
        MatrixAdd(IminusKalmanC,freeInovationTerm,this->P);
    }
    void ExtendedSystem::Predict()
    {
        MatrixMultiply(this->AE, this->DeltaStates, this->Xpred);
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[12][9],const  float secondMatrix[9][9],float (&resultMatrix)[12][9])
    {

         for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                resultMatrix[i][j] = 0;
                for (int k = 0; k < 9; k++)
                {
                    resultMatrix[i][j] += fistMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[12][12],const  float secondMatrix[12][9],float (&resultMatrix)[12][9])
    {

         for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                resultMatrix[i][j] = 0;
                for (int k = 0; k < 12; k++)
                {
                    resultMatrix[i][j] += fistMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[9][12],const  float secondMatrix[12],float (&resultMatrix)[9])
    {

        for (int i = 0; i < 9; i++)
        {
            resultMatrix[i] = 0;
            for (int j = 0; j < 12; j++)
            {
                resultMatrix[i] += fistMatrix[i][j] * secondMatrix[j];
            }
        }
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[12][12], const float secondMatrix[12], float (&resultMatrix)[12])
    {

        for (int i = 0; i < 12; i++)
        {
            resultMatrix[i] = 0;
            for (int j = 0; j < 12; j++)
            {
                resultMatrix[i] += fistMatrix[i][j] * secondMatrix[j];
            }
        }
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[9][12], const float secondMatrix[12][9], float (&resultMatrix)[9][9])
    {

        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                resultMatrix[i][j] = 0;
                for (int k = 0; k < 12; k++)
                {
                    resultMatrix[i][j] += fistMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[12][12],const  float secondMatrix[12][9],float (&resultMatrix)[9][12])
    {

        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                resultMatrix[i][j] = 0;
                for (int k = 0; k < 12; k++)
                {
                    resultMatrix[i][j] += fistMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[9][12], const float secondMatrix[12][12], float (&resultMatrix)[9][12])
    {

        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                resultMatrix[i][j] = 0;
                for (int k = 0; k < 12; k++)
                {
                    resultMatrix[i][j] += fistMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[12][9], const float secondMatrix[9], float (&resultMatrix)[12])
    {
        for (int i = 0; i < 12; i++)
        {
            resultMatrix[i] = 0;
            for (int j = 0; j < 9; j++)
            {
                resultMatrix[i] += fistMatrix[i][j] * secondMatrix[j];
            }
        }
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[12][5], const float secondMatrix[5], float (&resultMatrix)[12])
    {
        for (int i = 0; i < 12; i++)
        {
            resultMatrix[i] = 0;
            for (int j = 0; j < 5; j++)
            {
                resultMatrix[i] += fistMatrix[i][j] * secondMatrix[j];
            }
        }
    }

    void ExtendedSystem::Transpose(const float sourceMatrix[12][12], float (&resultMatrix)[12][12])
    {

        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                resultMatrix[i][j] = sourceMatrix[j][i];
            }
        }
    }

    void ExtendedSystem::Transpose(const float sourceMatrix[9][9], float (&resultMatrix)[9][9])
    {

        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                resultMatrix[i][j] = sourceMatrix[j][i];
            }
        }
    }

    void ExtendedSystem::Transpose(const float sourceMatrix[9][12], float (&resultMatrix)[12][9])
    {

        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                resultMatrix[i][j] = sourceMatrix[j][i];
            }
        }
    }

     void ExtendedSystem::Transpose(const float sourceMatrix[12][9], float (&resultMatrix)[9][12])
    {

        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                resultMatrix[i][j] = sourceMatrix[j][i];
            }
        }
    }

    void ExtendedSystem::MatrixMultiply(const float fistMatrix[12][12], const float secondMatrix[12][12], float (&resultMatrix)[12][12])
    {

        for (int i = 0; i < 12; i++)
        {

            for (int j = 0; j < 12; j++)
            {
                resultMatrix[i][j] = 0;
                for (int k = 0; k < 12; k++)
                {
                    resultMatrix[i][j] += fistMatrix[i][k] * secondMatrix[k][j];
                }
            }
        }
    }

    void ExtendedSystem::MatrixAdd(const float fistMatrix[12][12], const float secondMatrix[12][12], float (&resultMatrix)[12][12])
    {

        for (int i = 0; i < 12; i++)
        {
            for (int j = 0; j < 12; j++)
            {
                resultMatrix[i][j] += fistMatrix[i][j] + secondMatrix[i][j];
            }
        }
    }

    void ExtendedSystem::MatrixAdd(const float fistMatrix[12], const float secondMatrix[12], float (&resultMatrix)[12])
    {

        for (int i = 0; i < 12; i++)
        {
            resultMatrix[i] += fistMatrix[i] + secondMatrix[i];            
        }
    }

    void ExtendedSystem::MatrixAdd(const float fistMatrix[9][9], const float secondMatrix[9][9], float (&resultMatrix)[9][9])
    {

        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                resultMatrix[i][j] += fistMatrix[i][j] + secondMatrix[i][j];
            }
        }
    }

    void ExtendedSystem::Inverse(const float sourceMatrix[9][9], float (&resultMatrix)[9][9])
    {
        
        float det=determinantOfMatrix(sourceMatrix,9);
        if(det==0)
        {
            return;
        }
        Transpose(sourceMatrix,resultMatrix);
        for (int i = 0; i < 9; i++)
        {
            for (int j = 0; j < 9; j++)
            {
                resultMatrix[i][j] = resultMatrix[i][j] / det;
            }
        }
    }
    void subMatrix(const float mat[9][9], float temp[9][9], int p, int q, int n)
    {
        int i = 0, j = 0;
        // filling the sub matrix
        for (int row = 0; row < n; row++)
        {
            for (int col = 0; col < n; col++)
            {
                // skipping if the current row or column is not equal to the current
                // element row and column
                if (row != p && col != q)
                {
                    temp[i][j++] = mat[row][col];
                    if (j == n - 1)
                    {
                        j = 0;
                        i++;
                    }
                }
            }
        }
    }
    float determinantOfMatrix(const float matrix[9][9], float n)
    {
        float determinant = 0;
        if (n == 1)
        {
            return matrix[0][0];
        }
        if (n == 2)
        {
            return (matrix[0][0] * matrix[1][1]) - (matrix[0][1] * matrix[1][0]);
        }
        float temp[9][9], sign = 1;
        for (int i = 0; i < n; i++)
        {
            subMatrix(matrix, temp, 0, i, n);
            determinant += sign * matrix[0][i] * determinantOfMatrix(temp, n - 1);
            sign = -sign;
        }
        return determinant;
    }
}