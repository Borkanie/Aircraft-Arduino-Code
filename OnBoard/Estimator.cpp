#include "Estimator.h"
#include <iostream>

namespace Estimator
{
	float** dynamicAlocation(int n, int m)
	{
		float** result;
		result = new float* [n];

		for (int i = 0; i < n; i++)
		{
			result[i] = new float[m];
		}
		return result;
	}

	void deallocateMemo(float** result, int n, int m)
	{
		for (int i = 0; i < n; i++)
		{
			delete[] result[i];
		}
		delete[] result;
	}

	std::ostream& operator<<(std::ostream& os, const Matrix& dt) {
		os << "Matrix of size " << dt.rows << "," << dt.columns << ": " << std::endl;
		for (int i = 0; i < dt.rows; i++)
		{
			for (int j = 0; j < dt.columns; j++)
			{
				if ((dt.matrix[i][j] < 0.001 && dt.matrix[i][j]>0)||
					(dt.matrix[i][j] < 0 && dt.matrix[i][j]>-0.001))
				{
					os << 0 << ", ";
				}
				else {
					os << dt.matrix[i][j] << ", ";
				}
			}
			os << std::endl;
		}
		return os;
	}

#pragma region Integrator
	Integrator::Integrator() {
		SamplingTime = 0;
	}
	Integrator::Integrator(Matrix matrix, float samplingTime) {
		evaluatedValue = matrix;
		SamplingTime = samplingTime;
	}
	Integrator::Integrator(int n, int m, float samplingTime) {
		evaluatedValue = Matrix(n, m);
		SamplingTime = samplingTime;
	}
	void Integrator::ReadData(Matrix data) {
		evaluatedValue = evaluatedValue + data * SamplingTime;
	}
	Integrator::~Integrator() {
		evaluatedValue.~Matrix();
	}
	Matrix Integrator::GetValue() {
		return evaluatedValue;
	}
	Integrator& Integrator::operator=(const Integrator& other) {
		this->evaluatedValue = other.evaluatedValue;
		return *this;
	}
#pragma endregion
#pragma region IntegrationSystem
	IntegrationSystem::IntegrationSystem(const Matrix translation, const Matrix rotation) {
		translationalVelocities = Integrator(translation, 0.1);
		const float FSFControllerFix[5][12]{
			{0.9884, -0.0629, -0.2409, 0.0364, -1.4949, 0.7359, 0.2943, -3.4698, 0.7385, -0.9884, 0.0629, 0.2409},
			{0.0687, -0.0010, -0.0350, 0.0236, -1.2925, 0.1366, 0.0181, -1.4199, 0.1369, -0.0687, 0.0010, 0.0350},
			{-0.0008, 0.0501, 0.0001, -0.4123, 0.0053, -3.1612, -1.0159, 0.0104, -1.4229, 0.0008, -0.0501, -0.0001},
			{0.0065, -0.0297, -0.0078, 1.4937, -0.0358, 1.2694, 1.2559, -0.0489, 0.3442, -0.0065, 0.0297, 0.0078},
			{0.0058, 0.0323, -0.0076, -1.4853, -0.0291, -1.2804, -1.2620, -0.0377, -0.3510, -0.0058, -0.0323, 0.0076} };

		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->FSF).matrix[i][j] = FSFControllerFix[i][j];
			}
		}
	}

	IntegrationSystem::~IntegrationSystem() {
	}
	IntegrationSystem& IntegrationSystem::operator = (const IntegrationSystem& other)
	{
		this->translationalVelocities = other.translationalVelocities;
		this->rotationalVelocities = other.rotationalVelocities;
		return *this;
	}
	void IntegrationSystem::ReadTranslationalVelocities(const float accel[3]) {
		Matrix data(3, 1);
		for (int i = 0; i < 3; i++)
		{
			data.matrix[i][0] = accel[i];
		}
		translationalVelocities.ReadData(data);
	}
	void IntegrationSystem::ReadRoationalVelocities(const float accel[3]) {
		Matrix data(3, 1);
		for (int i = 0; i < 3; i++)
		{
			data.matrix[i][0] = accel[i];
		}
		rotationalVelocities.ReadData(data);
	}

#pragma endregion
#pragma region Matrix
	Matrix::Matrix() {
		this->rows = 1;
		this->columns = 1;
		this->matrix = dynamicAlocation(1, 1);
	}

	Matrix::Matrix(int n, int m) {
		this->rows = n;
		this->columns = m;
		this->matrix = dynamicAlocation(n, m);
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				matrix[i][j] = 0;
			}
		}
	}

	Matrix::Matrix(const Matrix& other) {
		this->matrix = dynamicAlocation(other.rows, other.columns);
		this->rows = other.rows;
		this->columns = other.columns;
		for (int i = 0; i < other.rows; i++)
		{
			for (int j = 0; j < other.columns; j++)
			{
				this->matrix[i][j] = other.matrix[i][j];
			}
		}
	}

	Matrix::~Matrix() {
		deallocateMemo(this->matrix, this->rows, this->columns);
	}

	Matrix Matrix::Transpose() {
		Matrix result(this->columns, this->rows);
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				result.matrix[j][i] = matrix[i][j];
			}
		}
		return result;
	}

	Matrix::Matrix(const Matrix& other, int startingRow, int endingRow, int startingColumn, int endingColumn) {
		this->matrix = dynamicAlocation(endingRow - startingRow, endingColumn - startingColumn);
		this->rows = endingRow - startingRow;
		this->columns = endingColumn - startingColumn;
		Attribute(other.matrix, startingRow, endingRow, startingColumn, endingColumn);
	}

	void Matrix::Attribute(float** sourceMatrix, int startingRow, int endingRow, int startingColumn, int endingColumn) {
		for (int i = 0; (i < rows) && (i + startingRow < endingRow); i++)
		{
			for (int j = 0; (j < columns) && (j + startingColumn < endingColumn); j++)
			{
				matrix[i][j] = sourceMatrix[i + startingRow][j + startingColumn];
			}
		}
	}

	void Matrix::CopyBloc(const Matrix bloc, int startingRow, int startingColumn, bool changeSign)
	{
		float sign = 1;
		if (changeSign)
		{
			sign = -1;
		}
		for (int i = 0; (i < bloc.rows) && (i + startingRow < rows); i++)
		{
			for (int j = 0; (j < bloc.columns) && (j + startingColumn < columns); j++)
			{
				matrix[i + startingRow][j + startingColumn] = sign * bloc.matrix[i][j];
			}
		}
	}

	Matrix Matrix::Determinant9x9()
	{
		Matrix A(*this, 0, 3, 0, 3);
		Matrix B(*this, 0, 3, 3, 9);
		Matrix C(*this, 3, 9, 0, 3);
		Matrix D(*this, 3, 9, 3, 9);
		Matrix invD = D.Inverse();
		Matrix invA = A.Inverse();
		Matrix Q((A - B * invD * C).Inverse());
		Matrix R((D - C * invA * B).Inverse());
		Matrix result(9, 9);
		result.CopyBloc(Q, 0, 0);
		result.CopyBloc(invD * C * Q, 3, 0, true);
		result.CopyBloc(invA * B * R, 0, 3, true);
		result.CopyBloc(R, 3, 3);
		return result;
	}

	Matrix Matrix::Determinant6x6()
	{
		Matrix A(*this, 0, 3, 0, 3);
		Matrix B(*this, 0, 3, 3, 6);
		Matrix C(*this, 3, 6, 0, 3);
		Matrix D(*this, 3, 6, 3, 6);
		Matrix invD = D.Inverse();
		Matrix invA = A.Inverse();
		Matrix Q((A - B * invD * C).Inverse());
		Matrix R((D - C * invA * B).Inverse());
		Matrix result(6, 6);
		result.CopyBloc(Q, 0, 0);
		result.CopyBloc(invD * C * Q, 3, 0, true);
		result.CopyBloc(invA * B * R, 0, 3, true);
		result.CopyBloc(R, 3, 3);
		return result;
	}

	Matrix Matrix::Inverse() {
		Matrix result(this->columns, this->rows);
		float det;
		if (this->columns == this->rows)
		{
			switch (this->columns)
			{
			case 9:
				result = Determinant9x9();
				break;
			case 6:
				result = Determinant6x6();
				break;
			case 3:
				det = Determinant();
				result.matrix[0][0] = (matrix[1][1] * matrix[2][2] - matrix[2][1] * matrix[1][2]) / det;
				result.matrix[1][0] = -(matrix[0][1] * matrix[2][2] - matrix[2][1] * matrix[0][2]) / det;
				result.matrix[2][0] = (matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2]) / det;

				result.matrix[0][1] = -(matrix[1][0] * matrix[2][2] - matrix[2][0] * matrix[1][2]) / det;
				result.matrix[1][1] = (matrix[0][0] * matrix[2][2] - matrix[2][0] * matrix[0][2]) / det;
				result.matrix[2][1] = -(matrix[0][0] * matrix[1][2] - matrix[1][0] * matrix[0][2]) / det;

				result.matrix[0][2] = (matrix[1][0] * matrix[2][1] - matrix[2][0] * matrix[1][1]) / det;
				result.matrix[1][2] = -(matrix[0][0] * matrix[2][1] - matrix[2][0] * matrix[0][1]) / det;
				result.matrix[2][2] = (matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1]) / det;
				result = result.Transpose();
				break;
			case 2:
				det = Determinant();
				result.matrix[0][0] = this->matrix[1][1] / det;
				result.matrix[0][1] = -this->matrix[0][1] / det;
				result.matrix[1][0] = -this->matrix[1][0] / det;
				result.matrix[1][1] = this->matrix[0][0] / det;
				break;
			case 1:
				det = Determinant();
				result.matrix[0][0] = 1 / det;
				break;
			default:
				break;
			}
		}

		return result;
	}

	float Matrix::Determinant() {
		float result = 1;

		if (this->columns == this->rows)
		{
			switch (this->columns)
			{
			case 3:
				return matrix[0][0] * matrix[1][1] * matrix[2][2]
					+ matrix[1][0] * matrix[2][1] * matrix[0][2]
					+ matrix[0][1] * matrix[1][2] * matrix[2][0]
					- matrix[2][0] * matrix[1][1] * matrix[0][2]
					- matrix[2][1] * matrix[1][2] * matrix[0][0]
					- matrix[1][0] * matrix[0][1] * matrix[2][2];
			case 2:
				return matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1];
			case 1:
				return matrix[0][0];
			default:
				break;
			}
		}

		return result;
	}

	Matrix& Matrix::operator = (const Matrix& other)
	{
		deallocateMemo(matrix, rows, columns);
		this->rows = other.rows;
		this->columns = other.columns;
		matrix = dynamicAlocation(rows, columns);
		for (int i = 0; i < other.rows; i++)
		{
			for (int j = 0; j < other.columns; j++)
			{
				this->matrix[i][j] = other.matrix[i][j];
			}
		}
		return *this;
	}

	Matrix Matrix::operator +(Matrix const& obj) {
		Matrix result(this->rows, obj.columns);
		for (int i = 0; i < this->rows; i++)
		{
			for (int j = 0; j < obj.columns; j++)
			{
				result.matrix[i][j] = this->matrix[i][j] + obj.matrix[i][j];
			}
		}
		return result;
	}

	Matrix Matrix::operator -(Matrix const& obj) {
		Matrix result(this->rows, this->columns);
		for (int i = 0; i < this->rows; i++)
		{
			for (int j = 0; j < this->columns; j++)
			{
				result.matrix[i][j] = this->matrix[i][j] - obj.matrix[i][j];
			}
		}
		return result;
	}

	Matrix Matrix::operator *(Matrix const& obj) {
		Matrix result(this->rows, obj.columns);
		for (int i = 0; i < this->rows; i++)
		{
			for (int j = 0; j < obj.columns; j++)
			{
				result.matrix[i][j] = 0;
				for (int k = 0; k < this->columns; k++)
				{
					result.matrix[i][j] += this->matrix[i][k] * obj.matrix[k][j];
				}
			}
		}
		return result;
	}

	Matrix Matrix::operator *(float const& scalar) {
		Matrix result(rows, columns);
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				result.matrix[i][j] = matrix[i][j] * scalar;
			}
		}
		return result;
	}

	void Matrix::ChangeSign()
	{
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < columns; j++)
			{
				matrix[i][j] = -matrix[i][j];
			}
		}
	}
#pragma endregion
#pragma region FixedSizeSystem
	FixedSizeSystem::~FixedSizeSystem()
	{
		Inputs.~Matrix();
		States.~Matrix();
		Ppred.~Matrix();
		DeltaStates.~Matrix();
		DeltaInputs.~Matrix();
		FSFController.~Matrix();
		KalmanGain.~Matrix();
		CE.~Matrix();
		BE.~Matrix();
		AE.~Matrix();
		R.~Matrix();
		Q.~Matrix();
		P.~Matrix();
		I12.~Matrix();
	}
	void FixedSizeSystem::DoKalmanAlgorithm(Matrix readings)
	{
		Xpred = AE * DeltaStates - BE * DeltaInputs;
		//std::cout << "Xpred= " << Xpred << std::endl;
		// A*p*A'+Q
		Ppred = AE * P * AE.Transpose() + Q;

		//std::cout << "AE= " << AE << std::endl;
		//std::cout << "Ppred= " << Ppred << std::endl;
		//Matrix temp = (CE * Ppred * CE.Transpose() + R);
		//KalmanGain = Ppred * CE.Transpose() * temp.Inverse();
		//std::cout << "KalmanGain= " << KalmanGain << std::endl;
		//std::cout << "CE * Ppred * CE.Transpose() + R= " << temp << std::endl;
		//std::cout << "(readings - States)  " << (readings - States)  << std::endl;
		DeltaStates = Xpred + KalmanGain * ((readings - States) - CE * Xpred);
		//std::cout << "DeltaStates= " << DeltaStates << std::endl;
		P = (I12 - KalmanGain * CE) * Ppred * (I12 - KalmanGain * CE).Transpose() + KalmanGain * R * KalmanGain.Transpose();
		//std::cout << "P= " << P << std::endl;
		DeltaInputs = FSFController * DeltaStates;
	}

	FixedSizeSystem::FixedSizeSystem()
	{
		InitConstants();
	}

	void FixedSizeSystem::InitConstants()
	{
		const float Kalman[12][9] = {
			{0.7631, -0.0043,    0.0090,    0.0010, -0.0193,    0.0043, -0.0007, -0.0608,    0.0065},
			{-0.0043,    0.8089,    0.0005,    0.0256, -0.0005, -0.0525, -0.0406, -0.0011, -0.0544},
			{0.0090,    0.0005,    0.8792, -0.0021,    0.0278, -0.0001,    0.0001,    0.0334, -0.0006},
			{0.0010,    0.0256, -0.0021,    0.6214,    0.0007,    0.0105,    0.0183,    0.0010,    0.0108},
			{-0.0193, -0.0005,    0.0278,    0.0007,    0.6088,    0.0004, -0.0001, -0.0035,    0.0007},
			{0.0043, -0.0525, -0.0001,    0.0105,    0.0004,    0.6011, -0.0118,    0.0013, -0.0072},
			{-0.0007, -0.0406,    0.0001,    0.0183, -0.0001, -0.0118,    0.6091, -0.0002, -0.0136},
			{-0.0608, -0.0011,    0.0334,    0.0010, -0.0035,    0.0013, -0.0002,    0.5910,    0.0021},
			{0.0065, -0.0544, -0.0006,    0.0108,    0.0007, -0.0072, -0.0136,    0.0021,    0.6002},
			{1.0691,    0.0063, -0.1243, -0.0544,    0.1650, -0.0372, -0.0006,    0.4899, -0.0555},
			{0.0337,    0.8834,    0.0056,    0.3923,    0.0018,    0.4236,    0.3036,    0.0098,    0.4210},
			{0.1341, -0.0128,    0.7603,    0.0166, -0.2574, -0.0106, -0.0038, -0.2785, -0.0084}
		};

		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 9; j++)
			{
				KalmanGain.matrix[i][j] = Kalman[i][j];
			}
		}

		for (int i = 0; i < 5; i++)
		{
			this->DeltaInputs.matrix[i][0] = 0;
		}

		for (int i = 0; i < 12; i++)
		{
			this->DeltaStates.matrix[i][0] = 0;
		}

		const float InputsFix[5] = { 5.5, 0, 0, 0, 0 };

		for (int i = 0; i < 5; i++)
		{
			this->Inputs.matrix[0][i] = InputsFix[i];
		}
		const float I[12][12] =
		{ {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1} };

		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->I12).matrix[i][j] = I[i][j];
			}
		}

		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->P).matrix[i][j] = I[i][j] * 100;
			}
		}

		const float QFix[12][12] = {
			{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1} };

		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->Q).matrix[i][j] = QFix[i][j];
			}
		}

		const float StatesFix[9] = { 11.8, 0, 1.8, 0, 0, 0, 0, 0.1, 0 };

		for (int i = 0; i < 9; i++)
		{
			(this->States).matrix[i][0] = StatesFix[i];
		}

		const float RFix[9][9] = {
			{1, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 1, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 1, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 1, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 1, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 1, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 1, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 1, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 1}
		};

		for (int i = 0; i < 9; i++)
		{
			for (int j = 0; j < 9; j++)
			{
				(this->R).matrix[i][j] = RFix[i][j];
			}
		}

		const float AEFix[12][12] = {
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
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000} };

		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->AE).matrix[i][j] = AEFix[i][j];
			}
		}

		const float BEFix[12][5] = {
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
			{0, 0, 0, 0, 0} };

		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 5; j++)
			{
				(this->BE).matrix[i][j] = BEFix[i][j];
			}
		}

		const float CEFix[9][12] = {
			{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0} };

		for (int i = 0; i < 9; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->CE).matrix[i][j] = CEFix[i][j];
			}
		}

		// Linear fullstate feedback controller for airspeed control.
		const float FSFControllerFix[5][12]{
			{0.9884, -0.0629, -0.2409, 0.0364, -1.4949, 0.7359, 0.2943, -3.4698, 0.7385, -0.9884, 0.0629, 0.2409},
			{0.0687, -0.0010, -0.0350, 0.0236, -1.2925, 0.1366, 0.0181, -1.4199, 0.1369, -0.0687, 0.0010, 0.0350},
			{-0.0008, 0.0501, 0.0001, -0.4123, 0.0053, -3.1612, -1.0159, 0.0104, -1.4229, 0.0008, -0.0501, -0.0001},
			{0.0065, -0.0297, -0.0078, 1.4937, -0.0358, 1.2694, 1.2559, -0.0489, 0.3442, -0.0065, 0.0297, 0.0078},
			{0.0058, 0.0323, -0.0076, -1.4853, -0.0291, -1.2804, -1.2620, -0.0377, -0.3510, -0.0058, -0.0323, 0.0076} };

		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->FSFController).matrix[i][j] = FSFControllerFix[i][j];
			}
		}
	}
#pragma endregion
#pragma region DynamicAllocation
	void DynamicAllocationSystem::DoKalmanAlgorithm(float readings[9])
	{
		this->Predict();
		this->CalculatePpred();
		this->CalculateNewGain();
		this->Estimate(readings);
		this->Innovate();
		deallocateMemo(this->DeltaInputs, 5, 1);
		this->DeltaInputs = Multiply(this->FSFController, 5, 12, this->DeltaStates, 12, 1);
	}
	void DynamicAllocationSystem::CalculatePpred()
	{
		// A*P
		float** result = Multiply(this->AE, 12, 12, this->P, 12, 12);

		// A'
		float** AETranspose = Transpose(this->AE, 12, 12);

		// A*P*A'
		float** result2 = Multiply(result, 12, 12, AETranspose, 12, 12);

		// Delete old pred matrix
		deallocateMemo(this->Ppred, 12, 12);
		// A*p*A'+Q
		this->Ppred = Add(result2, 12, 12, this->Q, 12, 12);

		//free up memory
		deallocateMemo(result2, 12, 12);
		deallocateMemo(AETranspose, 12, 12);
		deallocateMemo(result, 12, 12);
	}

	void DynamicAllocationSystem::CalculateNewGain()
	{
		// size(9,12).
		float** resultM1 = Multiply(this->CE, 9, 12, this->Ppred, 12, 12);

		// size(12,9).
		float** CET = Transpose(this->CE, 9, 12);

		// size(9,9)
		float** resultM2 = Multiply(resultM1, 9, 12, CET, 12, 9);

		// size(9,9)
		float** resultM3 = Add(resultM2, 9, 9, this->R, 9, 9);

		deallocateMemo(resultM2, 9, 9);
		resultM2 = inverse(resultM3, 9, 9);

		// size(12,9).
		float** resultM4 = Multiply(this->Ppred, 12, 12, CET, 12, 9);

		deallocateMemo(this->KalmanGain, 12, 9);
		this->KalmanGain = Multiply(resultM4, 12, 9, resultM2, 9, 9);

		//Clean up
		deallocateMemo(resultM2, 9, 9);
		deallocateMemo(CET, 12, 9);
		deallocateMemo(resultM4, 12, 9);
		deallocateMemo(resultM3, 9, 9);
		deallocateMemo(resultM1, 9, 12);
	}
	void DynamicAllocationSystem::Estimate(float readings[9])
	{
		//size(9,1)
		float** result1 = dynamicAlocation(9, 1);
		for (int i = 0; i < 9; i++)
		{
			result1[i][0] = (readings[i] - this->States[i][0]);
		}

		//size(9,1)
		float** result2 = Multiply(this->CE, 9, 12, this->Xpred, 12, 1);

		//size(9,1)
		float** paranthesis = Substract(result1, 9, 1, result2, 9, 1);

		//size(12,1)
		float** correction = Multiply(this->KalmanGain, 12, 9, paranthesis, 9, 1);

		deallocateMemo(this->DeltaStates, 12, 1);
		this->DeltaStates = Add(this->Xpred, 12, 1, correction, 12, 1);

		deallocateMemo(correction, 12, 1);
		deallocateMemo(result1, 9, 1);
		deallocateMemo(result2, 9, 1);
		deallocateMemo(paranthesis, 9, 1);
	}
	void DynamicAllocationSystem::Innovate()
	{
		// K*R
		// size(12,9)
		float** result1 = Multiply(this->KalmanGain, 12, 9, this->R, 9, 9);
		// K.' size(9,12)
		float** kalmanTranspose = Transpose(this->KalmanGain, 12, 9);
		// K*R*K'
		// K.' size(12,12)
		float** secondValue = Multiply(this->KalmanGain, 12, 9, kalmanTranspose, 9, 12);

		//size(12,12)
		float** KnCd = Multiply(this->KalmanGain, 12, 9, this->CE, 9, 12);
		//(eye(12)-Kn*Cd)
		//size(12,12)
		float** paranthesizTerms = Substract(I12, 12, 12, KnCd, 12, 12);
		//size(12,12)
		float** paranthesizTermsTranspose = Transpose(paranthesizTerms, 12, 12);
		//size(12,12)
		//(eye(12)-Kn*Cd)*Ppred
		float** tempValue = Multiply(paranthesizTerms, 12, 12, this->Ppred, 12, 12);
		//(eye(12)-Kn*Cd)*Ppred*(eye(12)-Kn*Cd).'
		float** firstValue = Multiply(tempValue, 12, 12, paranthesizTermsTranspose, 12, 12);

		// Update value in object
		deallocateMemo(this->P, 12, 12);
		this->P = Add(firstValue, 12, 12, secondValue, 12, 12);

		// Clean memory.
		deallocateMemo(firstValue, 12, 12);
		deallocateMemo(secondValue, 12, 12);
		deallocateMemo(result1, 12, 9);
		deallocateMemo(kalmanTranspose, 9, 12);
		deallocateMemo(paranthesizTerms, 12, 12);
		deallocateMemo(paranthesizTermsTranspose, 12, 12);
		deallocateMemo(tempValue, 12, 12);
	}
	void DynamicAllocationSystem::Predict()
	{
		deallocateMemo(this->Xpred, 12, 1);
		float** matrix1 = Multiply(this->AE, 12, 12, this->DeltaStates, 12, 1);
		float** matrix2 = Multiply(this->BE, 12, 5, this->DeltaInputs, 5, 1);
		this->Xpred = Add(matrix1, 12, 1, matrix2, 12, 1);
		deallocateMemo(matrix1, 12, 1);
		deallocateMemo(matrix2, 12, 1);
	}

	float** DynamicAllocationSystem::Transpose(float** A, int n1, int m1) {
		float** result = dynamicAlocation(m1, n1);
		for (int i = 0; i < n1; i++)
		{
			for (int j = 0; j < m1; j++)
			{
				result[j][i] = A[i][j];
			}
		}
		return result;
	}

	float** DynamicAllocationSystem::Multiply(float** matrix1, int n1, int m1, float** matrix2, int n2, int m2)
	{
		m1 == n2;
		float** result = dynamicAlocation(n1, m2);
		for (int i = 0; i < n1; i++)
		{
			for (int j = 0; j < m2; j++)
			{
				result[i][j] = 0;
				for (int k = 0; k < m1; k++)
				{
					result[i][j] += matrix1[i][k] * matrix2[k][j];
				}
			}
		}
		return result;
	}

	float** DynamicAllocationSystem::Add(float** matrix1, int n1, int m1, float** matrix2, int n2, int m2)
	{
		m1 == n2;
		float** result = dynamicAlocation(n1, m2);
		for (int i = 0; i < n1; i++)
		{
			for (int j = 0; j < m2; j++)
			{
				result[i][j] = matrix1[i][j] + matrix2[i][j];
			}
		}
		return result;
	}

	float** DynamicAllocationSystem::Substract(float** matrix1, int n1, int m1, float** matrix2, int n2, int m2)
	{
		m1 == n2;
		float** result = dynamicAlocation(n1, m2);
		for (int i = 0; i < n1; i++)
		{
			for (int j = 0; j < m2; j++)
			{
				result[i][j] = matrix1[i][j] - matrix2[i][j];
			}
		}
		return result;
	}

	// Function to get cofactor of A[p][q] in temp[][].n is current
	//  dimension of A[][]
	float** DynamicAllocationSystem::getCofactor(float** A, int n1, int m1, int p, int q)
	{
		int i = 0, j = 0;
		float** temp = dynamicAlocation(n1 - 1, m1 - 1);
		// Looping for each element of the matrix
		for (int row = 0; row < n1; row++)
		{
			for (int col = 0; col < m1; col++)
			{
				//  Copying into temporary matrix only those element
				//  which are not in given row and column
				if (row != p && col != q)
				{
					temp[i][j++] = A[row][col];

					// Row is filled, so increase row index and
					// reset col index
					if (j == n1 - 1)
					{
						j = 0;
						i++;
					}
				}
			}
		}
		return temp;
	}

	/* Recursive function for finding determinant of matrix.
	   n is current dimension of A[][]. */
	float DynamicAllocationSystem::determinant(float** A, int n1, int m1)
	{
		float D = 0; // Initialize result

		//  Base case : if matrix contains single element
		if (m1 == 1)
			return A[0][0];

		int sign = 1; // To store sign multiplier

		// Iterate for each element of first row
		for (int f = 0; f < m1; f++)
		{
			// Getting Cofactor of A[0][f]
			float** temp = getCofactor(A, n1, m1, 0, f);
			D += sign * A[0][f] * determinant(temp, n1 - 1, m1 - 1);
			deallocateMemo(temp, n1 - 1, m1 - 1);
			// terms are to be added with alternate sign
			sign = -sign;
		}

		return D;
	}

	// Function to get adjoint of A[N][N] in adj[N][N].
	float** DynamicAllocationSystem::adjoint(float** A, int n1, int m1)
	{
		float** temp = dynamicAlocation(n1, m1);
		if (n1 == 1)
		{
			temp[0][0] = 1;

			return temp;
		}

		// temp is used to store cofactors of A[][]
		int sign = 1;

		for (int i = 0; i < n1; i++)
		{
			for (int j = 0; j < m1; j++)
			{
				// Get cofactor of A[i][j]
				float** temp2 = getCofactor(A, n1, m1, i, j);

				// sign of adj[j][i] positive if sum of row
				// and column indexes is even.
				sign = ((i + j) % 2 == 0) ? 1 : -1;

				// Interchanging rows and columns to get the
				// transpose of the cofactor matrix
				temp[j][i] = (sign) * (determinant(temp2, n1 - 1, m1 - 1));
				deallocateMemo(temp2, n1 - 1, m1 - 1);
			}
		}
		return temp;
	}

	// Function to calculate and store inverse, returns false if
	// matrix is singular
	float** DynamicAllocationSystem::inverse(float** A, int n1, int m1)
	{
		// Find determinant of A[][]
		float det = determinant(A, n1, m1);

		// Find adjoint
		float** adj = adjoint(A, 9, 9);

		// Find Inverse using formula "inverse(A) = adj(A)/det(A)"
		for (int i = 0; i < n1; i++)
			for (int j = 0; j < m1; j++)
				adj[i][j] = adj[i][j] / det;

		return adj;
	}

	DynamicAllocationSystem::DynamicAllocationSystem()
	{
		InitConstants();
	}

	void DynamicAllocationSystem::InitConstants()
	{
		this->Xpred = dynamicAlocation(12, 1);
		this->DeltaInputs = dynamicAlocation(5, 1);
		for (int i = 0; i < 5; i++)
		{
			this->DeltaInputs[i][0] = 0;
		}
		this->DeltaStates = dynamicAlocation(12, 1);
		for (int i = 0; i < 12; i++)
		{
			this->DeltaStates[i][0] = 0;
		}
		this->Ppred = dynamicAlocation(12, 12);
		this->KalmanGain = dynamicAlocation(12, 9);

		const float InputsFix[5] = { 5.5, 0, 0, 0, 0 };
		this->Inputs = dynamicAlocation(1, 5);
		for (int i = 0; i < 5; i++)
		{
			this->Inputs[0][i] = InputsFix[i];
		}
		const float I[12][12] =
		{
			{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1} };
		(this->I12) = dynamicAlocation(12, 12);
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->I12)[i][j] = I[i][j];
			}
		}

		this->P = dynamicAlocation(12, 12);
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->P)[i][j] = I[i][j];
			}
		}

		const float QFix[12][12] = {
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
			{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} };
		(this->Q) = dynamicAlocation(12, 12);
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->Q)[i][j] = QFix[i][j];
			}
		}

		const float StatesFix[9] = { 11.8, 0, 1.8, 0, 0, 0, 0, 0.1, 0 };
		(this->States) = dynamicAlocation(9, 1);
		for (int i = 0; i < 9; i++)
		{
			(this->States)[i][0] = StatesFix[i];
		}

		const float RFix[9][9] = {
			{3, 3, 3, 3, 3, 3, 3, 3, 3},
			{3, 3, 3, 3, 3, 3, 3, 3, 3},
			{3, 3, 3, 3, 3, 3, 3, 3, 3},
			{3, 3, 3, 3, 3, 3, 3, 3, 3},
			{3, 3, 3, 3, 3, 3, 3, 3, 3},
			{3, 3, 3, 3, 3, 3, 3, 3, 3},
			{3, 3, 3, 3, 3, 3, 3, 3, 3},
			{3, 3, 3, 3, 3, 3, 3, 3, 3},
			{3, 3, 3, 3, 3, 3, 3, 3, 3} };
		(this->R) = dynamicAlocation(9, 9);
		for (int i = 0; i < 9; i++)
		{
			for (int j = 0; j < 9; j++)
			{
				(this->R)[i][j] = RFix[i][j];
			}
		}

		const float AEFix[12][12] = {
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
			{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0000} };
		(this->AE) = dynamicAlocation(12, 12);
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->AE)[i][j] = AEFix[i][j];
			}
		}

		const float BEFix[12][5] = {
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
			{0, 0, 0, 0, 0} };
		(this->BE) = dynamicAlocation(12, 5);
		for (int i = 0; i < 12; i++)
		{
			for (int j = 0; j < 5; j++)
			{
				(this->BE)[i][j] = BEFix[i][j];
			}
		}

		const float CEFix[9][12] = {
			{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0} };
		(this->CE) = dynamicAlocation(9, 12);
		for (int i = 0; i < 9; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->CE)[i][j] = CEFix[i][j];
			}
		}

		// Linear fullstate feedback controller for airspeed control.
		const float FSFControllerFix[5][12]{
			{0.9884, -0.0629, -0.2409, 0.0364, -1.4949, 0.7359, 0.2943, -3.4698, 0.7385, -0.9884, 0.0629, 0.2409},
			{0.0687, -0.0010, -0.0350, 0.0236, -1.2925, 0.1366, 0.0181, -1.4199, 0.1369, -0.0687, 0.0010, 0.0350},
			{-0.0008, 0.0501, 0.0001, -0.4123, 0.0053, -3.1612, -1.0159, 0.0104, -1.4229, 0.0008, -0.0501, -0.0001},
			{0.0065, -0.0297, -0.0078, 1.4937, -0.0358, 2.2694, 1.2559, -0.0489, 1.1442, -0.0065, 0.0297, 0.0078},
			{0.0058, 0.0323, -0.0076, -1.4853, -0.0291, -2.2804, -1.2620, -0.0377, -1.1510, -0.0058, -0.0323, 0.0076} };
		// Sampling time.
		(this->FSFController) = dynamicAlocation(5, 12);
		for (int i = 0; i < 5; i++)
		{
			for (int j = 0; j < 12; j++)
			{
				(this->FSFController)[i][j] = FSFControllerFix[i][j];
			}
		}
	}
#pragma endregion
}