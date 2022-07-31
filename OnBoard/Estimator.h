#pragma once
//#include <ostream>
#define DegreesToRadianstConst (3.18 / 180)
#define earthRadius 6371000
namespace Estimator
{
	// Returns the cosine relative to the radius that ends in Cluj-Napoca.
	float CosineInCluj(float angleInRadians);
	// Returns the sine relative to the radius that ends in Cluj-Napoca.
	float SineInCluj(float angleInRadians);
	// Dynamically allocates a matrix of n-lines and m-columns.
	float **dynamicAlocation(int n, int m);
	// Dynamically deletes a matrix of n-lines and m-columns.
	void deallocateMemo(float **result, int n, int m);
	/**
	 * A class that deals with matrix algebra because we couldn't use eigen on this controller.
	 */
	class Matrix
	{
	private:
		// Returns the determinant of a 6 by 6 matrix.
		// If our matrix is any bigger the target lines will be ignored.
		Matrix Determinant6x6();
		// Returns the determinant of a 9 by 9 matrix.
		// If our matrix is any bigger the target lines will be ignored.
		Matrix Determinant9x9();

	public:
		// Dynamically allocates a 1 by 1 matrix with the value of 0.
		Matrix();
		// Dynamically allocates a n by m matrix with the value of 0.
		Matrix(int n, int m);
		// Dynamically allocates a copy of the target matrix.
		Matrix(const Matrix &target);
		// Dynamically copies the values of the target matrix in a new copy.
		// @param startingRow The row on from which the new matrix will start.
		// @param endingRow The row on from which the new matrix will end.
		// @param startingColumn The column on from which the new matrix will start.
		// @param endingColumn); The column on from which the new matrix will start.
		Matrix(const Matrix &target, int startingRow, int endingRow, int startingColumn, int endingColumn);
		// The destructor that will  dynamically free up the memory used by the Matrix.
		~Matrix();
		int rows;
		int columns;
		// Pointer to Pointer to the first element of the matrix.
		float **matrix;
		Matrix &operator=(const Matrix &target);
		Matrix operator+(Matrix const &obj);
		Matrix operator*(Matrix const &obj);
		Matrix operator*(float const &scalar);
		Matrix operator-(Matrix const &obj);
		Matrix Transpose();
		Matrix Inverse();
		// Returns the determinant of a 1x2,2x2,3x3,6x6,9x9 or 12x12 matrix.
		float Determinant();
		// Changes the sign of all the lements in the matrix.
		void ChangeSign();
		// Copies the values of the target matrix in the current matrix.
		// @param startingRow The row on from which the new matrix will start.
		// @param endingRow The row on from which the new matrix will end.
		// @param startingColumn The column on from which the new matrix will start.
		// @param endingColumn); The column on from which the new matrix will start.
		void CopyBloc(const Matrix bloc, int startingRow, int startingColumn, bool changeSign = false);
		// Sets the values of the target matrix from the current matrix.
		// @param startingRow The row on from which the new matrix will start.
		// @param endingRow The row on from which the new matrix will end.
		// @param startingColumn The column on from which the new matrix will start.
		// @param endingColumn); The column on from which the new matrix will start.
		void Attribute(float **sourceMatrix, int startingRow, int endingRow, int startingColumn, int endingColumn);
	};
	// void subMatrix(const float mat[9][9], float temp[9][9], int p, int q, int n);
	// float determinantOfMatrix(const float matrix[9][9], float n);
	//  std::ostream& operator<<(std::ostream& os, const Matrix& dt);

	/**
	 * An implementation of an discrete time integrator with a fixed sammple time.
	 */
	class Integrator
	{
	private:
		float SamplingTime;
		// The current value.
		Matrix evaluatedValue;

	public:
		Integrator();
		// Creates an instances and sets matrix and sampling time.
		Integrator(const Matrix matrix, const float samplingTime);
		// Creates an instances with a new n-by-m matrix and sampling time.
		Integrator(const int n, const int m, const float samplingTime);
		// Returns the current value of the integrator.
		Matrix GetValue();
		// Feeds no data to the integrator. It should rtespect the predefined sampling time.
		void ReadData(Matrix data);
		Integrator &operator=(const Integrator &target);
		~Integrator();
	};

	/**
	 * A practical implementation of a discrete Proportional Integrating Derivating controller.
	 */
	class PIDz
	{
	private:
	public:
		float currentError = 0;
		float lastError = 0;
		// Proportional factor.
		float Kp = 0;
		// Integration factor.
		float Ki = 0;
		// Sampling Time.
		float Ts = 0;
		// Derivative factor.
		float Kd = 0;
		// Deriuvative filtering constant factor.
		float N = 0;
		PIDz();
		/**
		 * Creates a new PI Controller.
		 * @param kp Proportional factor
		 * @param ki Integration factor
		 * @param ts Sampling Time
		 */
		PIDz(float kp, float ki, float ts);
		/**
		 * Creates a new PID Controller.
		 * @param kp Proportional factor
		 * @param ki Integration factor
		 * @param ts Sampling Time
		 * @param kd Derivative factor
		 * @param d Deriuvative filtering constant factor.
		 */
		PIDz(float kp, float ki, float ts, float kd, float d);
		/**
		 * Creates a new PI Controller.
		 * @param startingValues Initial error.
		 * @param kp Proportional factor
		 * @param ki Integration factor
		 * @param ts Sampling Time
		 */
		PIDz(float startingValues, float kp, float ki, float ts);
		/**
		 * Creates a new PID Controller.
		 * @param startingValues Initial error.
		 * @param kp Proportional factor
		 * @param ki Integration factor
		 * @param ts Sampling Time
		 * @param kd Derivative factor
		 * @param d Deriuvative filtering constant factor.
		 */
		PIDz(float startingValues, float kp, float ki, float ts, float kd, float d);
		// Adds new error readings to the controller and recalculates command.
		void Read(float readValue);
		// Returns the current calculated command.
		float GetCommand();
		PIDz &operator=(const PIDz &target);
	};

	/**
	 * Implementation of differential equation system of an aircraft.
	 * It implements the integration step of differential set of equations.
	 * Also holds the value of a fullstate feedback controller.
	 * It expects access to translational and rotational accelerations from sensors and kalman.
	 */
	class IntegrationSystem
	{
	private:
		// Fullstate feedback controller.
		Matrix FSF = Matrix(5, 12);

	public:
		// An integrator regarding the translation movement of our aircraft.
		Integrator translationalVelocities;
		// An integrator regarding the rotational movement of our aircraft.
		Integrator rotationalVelocities;
		IntegrationSystem(const Matrix translation, const Matrix rotation);
		~IntegrationSystem();
		// Reads updates the translational velocieties with new translational accelerational values taken from sensors.
		void ReadTranslationalVelocities(const float accel[3]);
		// Reads updates the rotational velocieties with new rotational accelerational values taken from sensors.
		void ReadRoationalVelocities(const float accel[3]);
		IntegrationSystem &operator=(const IntegrationSystem &target);
	};

	/**
	 * A practical implementation of a discrete Proportional Integrating Derivating controller.
	 */
	class DynamicAllocationSystem
	{
	private:
		float **I12;

		// Our estimator actually estimates deviance from equilibrium conditions.
		// size(5,1).
		float **DeltaInputs;
		// size(12,1).
		float **DeltaStates;
		// size(12,12).
		float **Ppred;
		// The values of equilibrium for inputs.
		// size(5,1).
		float **Inputs;
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
		// All target states are irrelevant regarding translational movement in this model.
		// size(12,1).
		float **States;
		float **P;
		// Trust in model matrix size(12,12).
		float **Q;
		// Trust in readings matrics size(9,9).
		float **R;
		// Extended liniarized system matrix A size(12,12).
		float **AE;
		// Extended liniarized system matrix B size(12,5).
		float **BE;
		// Extended liniarized system matrix C size(9,12).
		float **CE;
		// size(12,9).
		float **KalmanGain;
		// Linear fullstate feedback controller for airspeed control.
		// size(5,12).
		float **FSFController;
		// Sampling time.
		const float Ts = 0.1;
		void Predict();
		void CalculatePpred();
		void CalculateNewGain();
		void Estimate(float readings[9]);
		void Innovate();
		// size(12,1)
		float **Xpred;
		float **Multiply(float **matrix1, int n1, int m1, float **matrix2, int n2, int m2);
		float **Add(float **matrix1, int n1, int m1, float **matrix2, int n2, int m2);
		float **Substract(float **matrix1, int n1, int m1, float **matrix2, int n2, int m2);
		float **getCofactor(float **A, int n1, int m1, int p, int q);
		float determinant(float **A, int n1, int m1);
		float **adjoint(float **A, int n1, int m1);
		float **inverse(float **A, int n1, int m1);
		float **Transpose(float **A, int n1, int m1);
		void InitConstants();

	public:
		DynamicAllocationSystem();
		void DoKalmanAlgorithm(float readings[9]);
	};

	/**
	 * Complete implementation of a discrete time linearization of differential set of equations governing an aircraft.
	 * Implements kalman filtering in order to estimate the states of our system.
	 */
	class FixedSizeSystem
	{
	private:
		Matrix I12 = Matrix(12, 12);
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
		/**
		*  The values of equilibrium for states.
		* The states are:
		* {
		* Velocity in X axis along body reference frame.
		* Velocity in Y axis along body reference frame.
		* Velocity in Z axis along body reference frame.
		* Angular velocity along X axis on body reference frame.
		* Angular velocity along Y axis on body reference frame.
		* Angular velocity along Z axis on body reference frame.
		* Euler angle X.
		* Euler angle Y.
		* Euler angle Z.
		* Wind velocity in X axis along body reference frame.
		* Wind velocity in Y axis along body reference frame.
		* Wind velocity in Z axis along body reference frame.
		* }
		* In order to stay in equilibrium the sum of velocity vectors must be null.
		* All target states are irrelevant regarding translational movement in this model.
		* size(12,1).
		*/
		Matrix States = Matrix(9, 1);
		~FixedSizeSystem();
	};
}