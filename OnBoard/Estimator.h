#pragma once
//#include <ostream>

namespace Estimator
{
	float** dynamicAlocation(int n, int m);
	void deallocateMemo(float** result, int n, int m);
	class Matrix {
	private:
		Matrix Determinant6x6();
		Matrix Determinant9x9();
	public:
		Matrix();
		Matrix(int n, int m);
		Matrix(const Matrix& other);
		Matrix(const Matrix& other, int startingRow, int endingRow, int startingColumn, int endingColumn);
		~Matrix();
		int rows;
		int columns;
		float** matrix;
		Matrix& operator =(const Matrix& other);
		Matrix operator + (Matrix const& obj);
		Matrix operator * (Matrix const& obj);
		Matrix operator * (float const& scalar);
		Matrix operator - (Matrix const& obj);
		Matrix Transpose();
		Matrix Inverse();
		float Determinant();
		void ChangeSign();
		void CopyBloc(const Matrix bloc, int startingRow, int startingColumn,bool changeSign=false);
		void Attribute(float** sourceMatrix, int startingRow, int endingRow, int startingColumn, int endingColumn);
	};
	void subMatrix(const float mat[9][9], float temp[9][9], int p, int q, int n);
	float determinantOfMatrix(const float matrix[9][9], float n);
	//std::ostream& operator<<(std::ostream& os, const Matrix& dt);

	class Integrator {
		private:
			float SamplingTime;
			Matrix evaluatedValue;
		public:
			Integrator();
			Integrator(const Matrix matrix, const float samplingTime);
			Integrator(const int n, const int m, const float samplingTime);
			Matrix GetValue();
			void ReadData(Matrix data);
			Integrator& operator=(const Integrator& other);
			~Integrator();
	};

	class PIDz {
	private:
	public:
		float currentError = 0;
		float lastError = 0;
		float Kp = 0;
		float Ki = 0;
		float Ts = 0;
		float Kd = 0;
		float N = 0;
		PIDz();
		PIDz(float kp, float ki, float ts);
		PIDz(float kp, float ki, float ts,float kd,float d);
		PIDz(float startingValues, float kp, float ki, float ts);
		PIDz(float startingValues, float kp, float ki, float ts, float kd, float d);
		void Read(float readValue);
		float GetCommand();
		PIDz& operator =(const PIDz& other);
	};

	class IntegrationSystem {
	private:
		Matrix FSF= Matrix(5,12);
	public:
		Integrator translationalVelocities;
		Integrator rotationalVelocities;
		IntegrationSystem(const Matrix translation, const Matrix rotation);
		~IntegrationSystem();
		void ReadTranslationalVelocities(const float accel[3]);
		void ReadRoationalVelocities(const float accel[3]);
		IntegrationSystem& operator =(const IntegrationSystem& other);
	};

	class DynamicAllocationSystem
	{
	private:
		float** I12;

		// Our estimator actually estimates deviance from equilibrium conditions.
		// size(5,1).
		float** DeltaInputs;
		// size(12,1).
		float** DeltaStates;
		// size(12,12).
		float** Ppred;
		// The values of equilibrium for inputs.
		// size(5,1).
		float** Inputs;
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
		// size(12,1).
		float** States;
		float** P;
		// Trust in model matrix size(12,12).
		float** Q;
		// Trust in readings matrics size(9,9).
		float** R;
		// Extended liniarized system matrix A size(12,12).
		float** AE;
		// Extended liniarized system matrix B size(12,5).
		float** BE;
		// Extended liniarized system matrix C size(9,12).
		float** CE;
		// size(12,9).
		float** KalmanGain;
		// Linear fullstate feedback controller for airspeed control.
		// size(5,12).
		float** FSFController;
		// Sampling time.
		const float Ts = 0.1;
		void Predict();
		void CalculatePpred();
		void CalculateNewGain();
		void Estimate(float readings[9]);
		void Innovate();
		// size(12,1)
		float** Xpred;
		float** Multiply(float** matrix1, int n1, int m1, float** matrix2, int n2, int m2);
		float** Add(float** matrix1, int n1, int m1, float** matrix2, int n2, int m2);
		float** Substract(float** matrix1, int n1, int m1, float** matrix2, int n2, int m2);
		float** getCofactor(float** A, int n1, int m1, int p, int q);
		float determinant(float** A, int n1, int m1);
		float** adjoint(float** A, int n1, int m1);
		float** inverse(float** A, int n1, int m1);
		float** Transpose(float** A, int n1, int m1);
		void InitConstants();

	public:
		DynamicAllocationSystem();
		void DoKalmanAlgorithm(float readings[9]);
	};

	class FixedSizeSystem
	{
	private:
		Matrix I12 = Matrix(12,12);
		// Our estimator actually estimates deviance from equilibrium conditions.
		Matrix P = Matrix(12, 12);
		// Trust in model matrix size(12,12).
		Matrix Q = Matrix(12, 12);
		// Trust in readings matrics size(9,9).
		Matrix R = Matrix(9, 9);
		// Extended liniarized system matrix A size(12,12).
		Matrix AE = Matrix(12, 12);
		// Extended liniarized system matrix B size(12,5).
		Matrix BE = Matrix(12, 5);
		// Extended liniarized system matrix C size(9,12).
		Matrix CE = Matrix(9, 12);
		// size(12,9).
		Matrix KalmanGain = Matrix(12, 9);
		// Linear fullstate feedback controller for airspeed control.
		// size(5,12).
		Matrix FSFController = Matrix(5, 12);
		// Sampling time.
		const float Ts = 0.1;
		// size(12,1)
		Matrix Xpred = Matrix(12, 1);
		void InitConstants();

	public:
		FixedSizeSystem();
		void DoKalmanAlgorithm(Matrix readings);
		// size(5,1).
		Matrix DeltaInputs = Matrix(5, 1);
		// size(12,1).
		Matrix DeltaStates = Matrix(12, 1);
		// size(12,12).
		Matrix Ppred = Matrix(12, 12);
		// The values of equilibrium for inputs.
		// size(5,1).
		Matrix Inputs = Matrix(5, 1);
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
		// size(12,1).
		Matrix States = Matrix(9, 1);
		~FixedSizeSystem();
	};
}