#pragma once

#include <utility>
#include <stdexcept>
#include <GPM/Tools/Utils.h>

// const Quaternion Quaternion::identity = Quaternion{ 0.0, 0.0, 0.0, 1.0 };

namespace GPM
{

#pragma region Constructors & Assignment
	inline Quaternion::Quaternion()
		: axis{ 0.0, 0.0, 0.0 }, w{ 1.0 }
	{	}

	inline Quaternion::Quaternion(const double p_x, const double p_y, const double p_z,
		const double p_w)
		: axis{ p_x, p_y, p_z }, w{ p_w }
	{	}

	inline Quaternion::Quaternion(const double p_scalar, const Vector3<double>& p_vector)
		: axis{ p_vector }, w{ p_scalar }
	{	}

	inline Quaternion::Quaternion(const Quaternion& p_other)
		: axis{ p_other.axis.x, p_other.axis.y, p_other.axis.z }, w{ p_other.w }
	{	}

	inline Quaternion::Quaternion(Quaternion&& p_other) noexcept
		: axis{ std::move(p_other.axis) }, w{ p_other.w }
	{	}

	inline Quaternion::Quaternion(const Matrix3<double>& p_matrix)
		: axis{ 0.0, 0.0, 0.0 }, w{ 1.0 }
	{
		const double trace = p_matrix.m_data[0] + p_matrix.m_data[4] + p_matrix.m_data[8];

		if (trace > 0.0f)
		{      //s=4*qw

			w = 0.5 * Tools::Utils::SquareRoot(1.0 + trace);
			const double S = 0.25 / w;

			axis.x = S * (p_matrix.m_data[5] - p_matrix.m_data[7]);
			axis.y = S * (p_matrix.m_data[6] - p_matrix.m_data[2]);
			axis.z = S * (p_matrix.m_data[1] - p_matrix.m_data[3]);

		}
		else if (p_matrix.m_data[0] > p_matrix.m_data[4] && p_matrix.m_data[0] > p_matrix.m_data[8])
		{ //s=4*qx

			axis.x = 0.5 * Tools::Utils::SquareRoot(1.0 + p_matrix.m_data[0] - p_matrix.m_data[4] - p_matrix.m_data[8]);
			const double X = 0.25 / axis.x;

			axis.y = X * (p_matrix.m_data[3] + p_matrix.m_data[1]);
			axis.z = X * (p_matrix.m_data[6] + p_matrix.m_data[2]);
			w = X * (p_matrix.m_data[5] - p_matrix.m_data[7]);
		}
		else if (p_matrix.m_data[4] > p_matrix.m_data[8])
		{ //s=4*qy

			axis.y = 0.5 * Tools::Utils::SquareRoot(1.0 - p_matrix.m_data[0] + p_matrix.m_data[4] - p_matrix.m_data[8]);
			const double Y = 0.25 / axis.y;
			axis.x = Y * (p_matrix.m_data[3] + p_matrix.m_data[1]);
			axis.z = Y * (p_matrix.m_data[7] + p_matrix.m_data[5]);
			w = Y * (p_matrix.m_data[6] - p_matrix.m_data[2]);

		}
		else
		{ //s=4*qz

			axis.z = 0.5 * Tools::Utils::SquareRoot(1.0 - p_matrix.m_data[0] - p_matrix.m_data[4] + p_matrix.m_data[8]);
			const double Z = 0.25 / axis.z;
			axis.x = Z * (p_matrix.m_data[6] + p_matrix.m_data[2]);
			axis.y = Z * (p_matrix.m_data[7] + p_matrix.m_data[5]);
			w = Z * (p_matrix.m_data[1] - p_matrix.m_data[3]);
		}
	}

	inline Quaternion::Quaternion(const Matrix4<double>& p_matrix)
		: axis{ 0.0, 0.0, 0.0 }, w{ 1.0 }
	{
		w = Tools::Utils::SquareRoot(std::max(0.0, 1.0 + p_matrix.m_data[0] + p_matrix.m_data[5] + p_matrix.m_data[10])) / 2.0;
		axis.x = Tools::Utils::SquareRoot(std::max(0.0, 1.0 + p_matrix.m_data[0] - p_matrix.m_data[5] - p_matrix.m_data[10])) / 2.0;
		axis.y = Tools::Utils::SquareRoot(std::max(0.0, 1.0 - p_matrix.m_data[0] + p_matrix.m_data[5] - p_matrix.m_data[10])) / 2.0;
		axis.z = Tools::Utils::SquareRoot(std::max(0.0, 1.0 - p_matrix.m_data[0] - p_matrix.m_data[5] + p_matrix.m_data[10])) / 2.0;

		axis.x *= Tools::Utils::Sign(axis.x * (p_matrix.m_data[9] - p_matrix.m_data[6]));
		axis.y *= Tools::Utils::Sign(axis.y * (p_matrix.m_data[2] - p_matrix.m_data[8]));
		axis.z *= Tools::Utils::Sign(axis.z * (p_matrix.m_data[4] - p_matrix.m_data[1]));
	}

	inline Quaternion::Quaternion(const Vector3<double>& p_axis,
		const double p_angleInRadians)
		: axis{ 0.0, 0.0, 0.0 }, w{ 1.0 }
	{
		const double angleDivided = p_angleInRadians / 2.0;

		w = Tools::Utils::Cos(angleDivided);

		const double sinAngle = Tools::Utils::Sin(angleDivided);

		axis.x = sinAngle * p_axis.x;
		axis.y = sinAngle * p_axis.y;
		axis.z = sinAngle * p_axis.z;
	}

	inline void Quaternion::MakeFromEuler(const Vector3<double>& p_euler)
	{
		double x = Tools::Utils::ToRadians(p_euler.x);
		double y = Tools::Utils::ToRadians(p_euler.y);
		double z = Tools::Utils::ToRadians(p_euler.z);

		x = x / 2.0;
		y = y / 2.0;
		z = z / 2.0;

		w = cos(z) * cos(y) * cos(x) + sin(z) * sin(y) * sin(x);
		axis.x = cos(z) * cos(y) * sin(x) - sin(z) * sin(y) * cos(x);
		axis.y = cos(z) * sin(y) * cos(x) + sin(z) * cos(y) * sin(x);
		axis.z = sin(z) * cos(y) * cos(x) - cos(z) * sin(y) * sin(x);
	}

	inline void Quaternion::MakeFromEuler(const double p_x, const double p_y, const double p_z)
	{
		double x = Tools::Utils::ToRadians(p_x);
		double y = Tools::Utils::ToRadians(p_y);
		double z = Tools::Utils::ToRadians(p_z);

		x = x / 2.0;
		y = y / 2.0;
		z = z / 2.0;

		w = cos(z) * cos(y) * cos(x) + sin(z) * sin(y) * sin(x);
		axis.x = cos(z) * cos(y) * sin(x) - sin(z) * sin(y) * cos(x);
		axis.y = cos(z) * sin(y) * cos(x) + sin(z) * cos(y) * sin(x);
		axis.z = sin(z) * cos(y) * cos(x) - cos(z) * sin(y) * sin(x);
	}

	inline Quaternion& Quaternion::operator=(Quaternion&& p_other) noexcept
	{
		w = p_other.w;
		axis = p_other.axis;

		return (*this);
	}

	inline bool Quaternion::IsIdentity() const
	{
		return axis.x == 0.0 && axis.y == 0.0 && axis.z == 0.0;
	}

	inline bool Quaternion::IsPure() const
	{
		return w == 0.0;
	}

	inline bool Quaternion::IsNormalized() const
	{
		return Norm() == 1.0;
	}

	inline bool Quaternion::operator==(const Quaternion& p_otherQuaternion) const
	{
		return w == p_otherQuaternion.w && axis == p_otherQuaternion.axis;
	}

	inline bool Quaternion::operator!=(const Quaternion& p_otherQuaternion) const
	{
		return w != p_otherQuaternion.w || axis != p_otherQuaternion.axis;
	}

	inline Quaternion Quaternion::operator+(const Quaternion& p_otherQuaternion) const
	{
		return  { Quaternion{  w + p_otherQuaternion.w, axis + p_otherQuaternion.axis } };
	}

	inline Quaternion& Quaternion::operator+=(const Quaternion& p_otherQuaternion)
	{
		w += p_otherQuaternion.w;
		axis += p_otherQuaternion.axis;

		return { *this };
	}

	inline Quaternion Quaternion::operator-(const Quaternion& p_otherQuaternion) const
	{
		return  { Quaternion{  w - p_otherQuaternion.w, axis - p_otherQuaternion.axis } };
	}

	inline Quaternion& Quaternion::operator-=(const Quaternion& p_otherQuaternion)
	{
		w -= p_otherQuaternion.w;
		axis -= p_otherQuaternion.axis;

		return { *this };
	}

	inline double Quaternion::DotProduct(const Quaternion& p_otherQuaternion) const
	{
		return w * p_otherQuaternion.w + axis.x * p_otherQuaternion.axis.x + axis.y * p_otherQuaternion.axis.y + axis.z * p_otherQuaternion.axis.z;
	}

	inline double Quaternion::DotProduct(const Quaternion& p_left, const Quaternion& p_right)
	{
		return p_left.w * p_right.w + p_left.axis.x * p_right.axis.x + p_left.axis.y * p_right.axis.y + p_left.axis.z * p_right.axis.z;
	}

	inline Quaternion& Quaternion::operator*=(const double p_scale)
	{
		w *= p_scale;
		axis *= p_scale;

		return { *this };
	}

	inline Quaternion Quaternion::operator*(const double p_scale) const
	{
		return { Quaternion{w * p_scale, axis * p_scale} };
	}

	inline Quaternion Quaternion::operator*(const Quaternion& p_otherQuaternion) const
	{
		return { (*this).Multiply(p_otherQuaternion) };
	}

	inline Quaternion& Quaternion::operator*=(const Quaternion& p_otherQuaternion)
	{
		(*this) = Multiply(p_otherQuaternion);

		return { (*this) };
	}

	inline Quaternion Quaternion::operator*(const Vector3<double>& p_toMultiply) const
	{
		const double sPart = -(axis.x * p_toMultiply.x + axis.y * p_toMultiply.y + axis.z * p_toMultiply.z);
		const double xPart = w * p_toMultiply.x + axis.y * p_toMultiply.z - axis.z * p_toMultiply.y;
		const double yPart = w * p_toMultiply.y + axis.z * p_toMultiply.x - axis.x * p_toMultiply.z;
		const double zPart = w * p_toMultiply.z + axis.x * p_toMultiply.y - axis.y * p_toMultiply.x;

		return { Quaternion{ sPart, Vector3<double> { xPart, yPart, zPart } } };
	}

	inline Quaternion& Quaternion::operator*=(const Vector3<double>& p_toMultiply)
	{
		const double sPart = -(axis.x * p_toMultiply.x + axis.y * p_toMultiply.y + axis.z * p_toMultiply.z);
		const double xPart = w * p_toMultiply.x + axis.y * p_toMultiply.z - axis.z * p_toMultiply.y;
		const double yPart = w * p_toMultiply.y + axis.z * p_toMultiply.x - axis.x * p_toMultiply.z;
		const double zPart = w * p_toMultiply.z + axis.x * p_toMultiply.y - axis.y * p_toMultiply.x;

		w = sPart;
		axis = Vector3<double>{ xPart, yPart, zPart };

		return { (*this) };
	}

	inline Quaternion Quaternion::Multiply(const Quaternion& p_quaternion) const
	{
		Quaternion result;
		result.axis.x = axis.x * p_quaternion.w + axis.y * p_quaternion.axis.z - axis.z * p_quaternion.axis.y + w * p_quaternion.axis.x;
		result.axis.y = -axis.x * p_quaternion.axis.z + axis.y * p_quaternion.w + axis.z * p_quaternion.axis.x + w * p_quaternion.axis.y;
		result.axis.z = axis.x * p_quaternion.axis.y - axis.y * p_quaternion.axis.x + axis.z * p_quaternion.w + w * p_quaternion.axis.z;
		result.w = -axis.x * p_quaternion.axis.x - axis.y * p_quaternion.axis.y - axis.z * p_quaternion.axis.z + w * p_quaternion.w;
		return { result };

	}

	inline double Quaternion::Norm() const
	{
		return Tools::Utils::SquareRoot(w * w + axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
	}

	inline Quaternion& Quaternion::Inverse()
	{
		double absoluteValue = NormSquare();
		absoluteValue = 1.0 / absoluteValue;

		const Quaternion conjugateValue = Conjugate();

		const double scalar = conjugateValue.w * (absoluteValue);
		const Vector3<double> imaginary = conjugateValue.axis * (absoluteValue);

		w = scalar;
		axis = imaginary;

		return { (*this) };
	}

	inline Quaternion Quaternion::Inverse(const Quaternion& p_quaternion)
	{
		double absoluteValue = p_quaternion.NormSquare();
		absoluteValue = 1.0 / absoluteValue;

		const Quaternion conjugateValue = Conjugate(p_quaternion);

		return { Quaternion {conjugateValue.w * absoluteValue, Vector3<double> {conjugateValue.axis* absoluteValue} } };
	}

	inline Quaternion& Quaternion::Conjugate()
	{
		axis *= -1.0;

		return { (*this) };
	}

	inline Quaternion Quaternion::Conjugate(const Quaternion& p_quaternion)
	{
		return { Quaternion { p_quaternion.w, p_quaternion.axis * -1.0 } };
	}

	inline Quaternion& Quaternion::ConvertToUnitNormQuaternion()
	{
		const double angle = Tools::Utils::ToRadians(w);

		axis.Normalize();
		w = Tools::Utils::Cos(angle * 0.5);
		axis = axis * Tools::Utils::Sin(angle * 0.5);

		return { (*this) };
	}

	inline Vector3<double> Quaternion::RotateVectorAboutAngleAndAxis(const double p_angle, const Vector3<double>& p_axis, const Vector3<double>& p_vectorToRotate)
	{
		const Quaternion p{ 0, p_vectorToRotate };

		//normalize the axis
		const Vector3<double> uAxis = p_axis.Normalized();

		//create the real quaternion
		Quaternion q{ p_angle, uAxis };

		//convert quaternion to unit norm quaternion
		q.ConvertToUnitNormQuaternion();

		const Quaternion qInverse = Inverse(q);

		const Quaternion rotatedVector = q * p * qInverse;

		return rotatedVector.axis;
	}

	inline double Quaternion::operator[](const int p_index) const
	{
		if (p_index < 0 || p_index > 3)
			throw std::out_of_range("Out of range access with index:" + std::to_string(p_index) + " in Quaternion");

		switch (p_index)
		{
		case 0: return w;
		case 1: return axis.x;
		case 2: return axis.y;
		case 3: return axis.z;
		default: return 1.0;
		}
	}

	inline Vector3<double> Quaternion::GetRotationAxis() const
	{
		return axis;
	}

	inline double Quaternion::GetXAxisValue() const
	{
		return axis.x;
	}

	inline double Quaternion::GetYAxisValue() const
	{
		return axis.y;
	}

	inline double Quaternion::GetZAxisValue() const
	{
		return axis.z;
	}

	inline double Quaternion::GetRealValue() const
	{
		return w;
	}

	inline void Quaternion::SetXAxisValue(const double p_xValue)
	{
		axis.x = p_xValue;
	}

	inline void Quaternion::SetYAxisValue(const double p_yValue)
	{
		axis.y = p_yValue;
	}

	inline void Quaternion::SetZAxisValue(const double p_zValue)
	{
		axis.z = p_zValue;
	}

	inline void Quaternion::SetRealValue(const double p_realValue)
	{
		w = p_realValue;
	}

	inline Quaternion Quaternion::LookRotation(const Vector3<double>& p_forward, const Vector3<double>& p_upwards) const
	{
		const Vector3<double> forwardVector = (p_upwards - p_forward).Normalized();

		const double dot = Vector3<double>::forward.Dot(forwardVector);

		if (Tools::Utils::Abs<double>(dot - (-1.0)) < 0.000001)
		{
			return Quaternion(Vector3<double>::up.x, Vector3<double>::up.y, Vector3<double>::up.z, static_cast<double>(Tools::M_PI));
		}
		if (Tools::Utils::Abs<double>(dot - (1.0)) < 0.000001)
		{
			return Quaternion{ 0.0, 0.0, 0.0, 1.0 };
		}

		const double rotAngle = Tools::Utils::Arccos(dot);
		Vector3<double> rotAxis = Vector3<double>::Cross(Vector3<double>::forward, forwardVector);
		rotAxis = rotAxis.Normalized();
		return CreateFromAxisAngle(rotAxis, rotAngle);
	}

	inline Quaternion Quaternion::CreateFromAxisAngle(const Vector3<double>& p_axis,
		const double p_angle)
	{
		const double halfAngle = p_angle * 0.5;
		const double s = Tools::Utils::Sin(halfAngle);

		Quaternion q;
		q.axis.x = p_axis.x * s;
		q.axis.y = p_axis.y * s;
		q.axis.z = p_axis.z * s;
		q.w = Tools::Utils::Cos(halfAngle);

		return q;
	}

	inline Quaternion Quaternion::Lerp(const Quaternion& p_start, const Quaternion& p_end,
		const double p_alpha)
	{
		const double coefficient = 1.0 - p_alpha;

		return  { Quaternion { coefficient * p_start.axis.x + p_alpha * p_end.axis.x,
							coefficient * p_start.axis.y + p_alpha * p_end.axis.y,
							coefficient * p_start.axis.z + p_alpha * p_end.axis.z,
							coefficient * p_start.w + p_alpha * p_end.w } // .Normalize(); // Cancel the interpolation ?
		};
	}

	inline Quaternion Quaternion::Slerp(const Quaternion& p_start, const Quaternion& p_end,
		const double p_alpha)
	{
		const Quaternion qStartNormalized = Normalize(p_start);
		const Quaternion qEndNormalized = Normalize(p_end);

		double dot = DotProduct(qStartNormalized, qEndNormalized);

		//clamp values (just in case) because ArcCos only works from -1 to 1
		if (dot > 1.0)
		{
			dot = 1.0;
		}
		else if (dot < -1.0)
			dot = -1.0;

		const double theta = Tools::Utils::Arccos(dot) * p_alpha;
		Quaternion relativeQuaternion = qEndNormalized - qStartNormalized * dot;
		relativeQuaternion.Normalize();

		Quaternion result = qStartNormalized * Tools::Utils::Cos(theta) + relativeQuaternion * Tools::Utils::Sin(theta);

		return result;
	}

	inline Quaternion Quaternion::SlerpShortestPath(const Quaternion& p_start, const Quaternion& p_end, const double p_alpha)
	{
		Quaternion qStartNormalized = Normalize(p_start);
		const Quaternion qEndNormalized = Normalize(p_end);

		double dot = DotProduct(qStartNormalized, qEndNormalized);

		// If the dot product is negative,
		// Slerp will not look for the closest rotation -> It will spin the other way around.
		if (dot < 0.0)
		{
			qStartNormalized.w = -qStartNormalized.w;
			qStartNormalized.axis.x = -qStartNormalized.axis.x;
			qStartNormalized.axis.y = -qStartNormalized.axis.y;
			qStartNormalized.axis.z = -qStartNormalized.axis.z;
			dot = -dot;
		}

		//clamp values (just in case) because ArcCos only works from -1 to 1
		if (dot > 1.0)
		{
			dot = 1.0;
		}
		else if (dot < -1.0)
			dot = -1.0;

		const double theta = Tools::Utils::Arccos(dot) * p_alpha;
		Quaternion relativeQuaternion = qEndNormalized - qStartNormalized * dot;
		relativeQuaternion.Normalize();

		Quaternion result = qStartNormalized * Tools::Utils::Cos(theta) + relativeQuaternion * Tools::Utils::Sin(theta);

		return result;
	}

	inline Quaternion Quaternion::Nlerp(const Quaternion& p_start,
		const Quaternion& p_end, const double p_alpha)
	{
		return Lerp(p_start, p_end, p_alpha).Normalize();
	}

	constexpr double Quaternion::NormSquare() const
	{
		return w * w + axis.x * axis.x + axis.y * axis.y + axis.z * axis.z;
	}

	inline Quaternion& Quaternion::Normalize()
	{
		if (Norm() > 0.0) {
			const double normValue = 1.0 / Norm();

			w *= normValue;
			axis *= normValue;
		}

		return { (*this) };
	}

	inline Quaternion Quaternion::Normalize(const Quaternion& p_quaternion)
	{
		double scalar = 0.0;

		Vector3<double> vector{};

		if (p_quaternion.Norm() != 0.0) {
			const double normValue = 1.0 / p_quaternion.Norm();

			scalar = p_quaternion.w * normValue;
			vector = p_quaternion.axis * normValue;
		}

		return { Quaternion{ scalar, vector} };
	}

	inline Quaternion Quaternion::ToUnitNormQuaternion()
	{
		const double angle = Tools::Utils::ToRadians(w);

		axis.Normalize();

		return { Quaternion { Tools::Utils::Cos(angle * 0.5), axis * Tools::Utils::Sin(angle * 0.5)} };
	}

	inline Vector3<double> Quaternion::ToEuler() const
	{
		Vector3<double> euler{};

		// roll (x-axis rotation)
		const double sinr_cosp = 2.0 * (w * axis.x + axis.y * axis.z);
		const double cosr_cosp = 1.0 - 2.0 * (axis.x * axis.x + axis.y * axis.y);
		euler.x = Tools::Utils::Arctan2(sinr_cosp, cosr_cosp);

		// pitch (y-axis rotation)
		const double sinp = 2.0 * (w * axis.y - axis.z * axis.x);
		if (Tools::Utils::Abs(sinp) >= 1.0)
			euler.y = std::copysign(Tools::M_PI / 2.0, sinp); // use 90 degrees if out of range
		else
			euler.y = Tools::Utils::Arcsin(sinp);

		// yaw (z-axis rotation)
		const double siny_cosp = 2.0 * (w * axis.z + axis.x * axis.y);
		const double cosy_cosp = 1.0 - 2.0 * (axis.y * axis.y + axis.z * axis.z);
		euler.z = Tools::Utils::Arctan2(siny_cosp, cosy_cosp);

		euler.x = Tools::Utils::ToDegrees(euler.x);
		euler.y = Tools::Utils::ToDegrees(euler.y);
		euler.z = Tools::Utils::ToDegrees(euler.z);

		return euler;
	}

	inline Quaternion Quaternion::ToQuaternion(const Vector3<double>& p_euler)
	{
		return { ToQuaternion(p_euler.x, p_euler.y, p_euler.z) };
	}

	inline Quaternion Quaternion::ToQuaternion(const double p_yaw, const double p_pitch, const double p_roll)
	{
		Quaternion result;

		const double cosYaw = Tools::Utils::Cos(p_yaw * 0.5);
		const double sinYaw = Tools::Utils::Sin(p_yaw * 0.5);
		const double cosPitch = Tools::Utils::Cos(p_pitch * 0.5);
		const double sinPitch = Tools::Utils::Sin(p_pitch * 0.5);
		const double cosRoll = Tools::Utils::Cos(p_roll * 0.5);
		const double sinRoll = Tools::Utils::Sin(p_roll * 0.5);

		result.w = cosYaw * cosPitch * cosRoll + sinYaw * sinPitch * sinRoll;
		result.axis.x = cosYaw * cosPitch * sinRoll - sinYaw * sinPitch * cosRoll;
		result.axis.y = sinYaw * cosPitch * sinRoll + cosYaw * sinPitch * cosRoll;
		result.axis.z = sinYaw * cosPitch * cosRoll - cosYaw * sinPitch * sinRoll;

		return { result };
	}

	inline std::string Quaternion::ToString() const
	{
		return { std::string("(w: " + std::to_string(w) + "; x: " + std::to_string(axis.x) + ", y: " + std::to_string(axis.y) +
			", z: " + std::to_string(axis.z)) + ')' };
	}

	inline std::string Quaternion::ToString(const Quaternion& p_quaternion)
	{
		return { p_quaternion.ToString() };
	}

	inline Matrix3<float> Quaternion::ToMatrix3() const
	{
		Matrix3<float> result;

		const float fw = static_cast<float>(w);
		Vector3<float> faxis{};
		faxis.x = static_cast<float>(axis.x);
		faxis.y = static_cast<float>(axis.y);
		faxis.z = static_cast<float>(axis.z);

		result.m_data[0] = 2.0f * (fw * fw + faxis.x * faxis.x) - 1.0f;
		result.m_data[3] = 2.0f * (faxis.x * faxis.y - fw * faxis.z);
		result.m_data[6] = 2.0f * (faxis.x * faxis.z + fw * faxis.y);

		result.m_data[1] = 2.0f * (faxis.x * faxis.y + fw * faxis.z);
		result.m_data[4] = 2.0f * (fw * fw + faxis.y * faxis.y) - 1.0f;
		result.m_data[7] = 2.0f * (faxis.y * faxis.z - fw * faxis.x);

		result.m_data[2] = 2.0f * (faxis.x * faxis.z - fw * faxis.y);
		result.m_data[5] = 2.0f * (faxis.y * faxis.z + fw * faxis.x);
		result.m_data[8] = 2.0f * (fw * fw + faxis.z * faxis.z) - 1.0f;

		return result;
	}

	inline Matrix4<float> Quaternion::ToMatrix4() const
	{
		Matrix4<float> result{};
		const float sqw = static_cast<float>(w * w);
		const float sqx = static_cast<float>(axis.x * axis.x);
		const float sqy = static_cast<float>(axis.y * axis.y);
		const float sqz = static_cast<float>(axis.z * axis.z);

		// invs (inverse square length) is only required if quaternion is not already normalised
		const float invs = 1.0f / (sqx + sqy + sqz + sqw);
		result.m_data[0] = (sqx - sqy - sqz + sqw) * invs; // since sqw + sqx + sqy + sqz =1/invs*invs
		result.m_data[5] = (-sqx + sqy - sqz + sqw) * invs;
		result.m_data[10] = (-sqx - sqy + sqz + sqw) * invs;

		float tmp1 = static_cast<float>(axis.x * axis.y);
		float tmp2 = static_cast<float>(axis.z * w);
		result.m_data[4] = 2.0f * (tmp1 + tmp2) * invs;
		result.m_data[1] = 2.0f * (tmp1 - tmp2) * invs;

		tmp1 = static_cast<float>(axis.x * axis.z);
		tmp2 = static_cast<float>(axis.y * w);
		result.m_data[8] = 2.0f * (tmp1 - tmp2) * invs;
		result.m_data[2] = 2.0f * (tmp1 + tmp2) * invs;
		tmp1 = static_cast<float>(axis.y * axis.z);
		tmp2 = static_cast<float>(axis.x * w);
		result.m_data[9] = 2.0f * (tmp1 + tmp2) * invs;
		result.m_data[6] = 2.0f * (tmp1 - tmp2) * invs;

		return { result };
	}

	inline std::ostream& operator<<(std::ostream& p_stream,
		const Quaternion& p_quaternion)
	{
		p_stream << "(w: " << p_quaternion.w << "; x: " << p_quaternion.axis.x << ", y: " << p_quaternion.axis.y <<
			", z: " << p_quaternion.axis.z << ')';
		return  { p_stream };
	}
}
#pragma endregion