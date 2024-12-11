#include "Quaternion.h"
#include "Math//MathFunction.h"
#include "Novice.h"

//間隔
static const int kRowHeight = 20;
static const int kColumnWidth = 60;

using namespace Math;

Quaternion Quaternion::Multiply(const Quaternion& lhs, const Quaternion& rhs)
{
	Quaternion result{};
	result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
	result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
	result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
	result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
	return result;
}

Quaternion Quaternion::IdentityQuaternion()
{
	return Quaternion{ 0.0f, 0.0f, 0.0f, 1.0f };
}

Quaternion Quaternion::Conjugate(const Quaternion& quaternion)
{
	return Quaternion{ -quaternion.x, -quaternion.y, -quaternion.z, quaternion.w };
}

float Quaternion::Norm(const Quaternion& quaternion)
{
	return sqrtf(pow(quaternion.x, 2.0f) + pow(quaternion.y, 2.0f) + pow(quaternion.z, 2.0f) + pow(quaternion.w, 2.0f));
}

Quaternion Quaternion::Normalize(const Quaternion& quaternion)
{
	float norm = Norm(quaternion);
	if (norm == 0.0f)
	{
		return IdentityQuaternion();
	}

	return Quaternion{ quaternion.x / norm, quaternion.y / norm, quaternion.z / norm, quaternion.w / norm };
}

float Quaternion::Dot(const Quaternion& q1, const Quaternion& q2)
{
	return q1.w * q2.w + q1.x * q2.x + q1.y * q2.y + q2.z * q2.z;
}

Quaternion Quaternion::Inverse(const Quaternion& quaternion)
{
	float normSquared = pow(Norm(quaternion), 2.0f);
	if (normSquared == 0.0f)
	{
		return IdentityQuaternion();
	}

	Quaternion conjugate = Conjugate(quaternion);

	return Quaternion{ conjugate.x / normSquared, conjugate.y / normSquared, conjugate.z / normSquared, conjugate.w / normSquared };
}

Quaternion Quaternion::MakeRotateAxisAngleQuaternion(const Vector3& axis, float angle)
{
	Quaternion result;
	float halfAngle = angle / 2.0f;
	float sinHalfAngle = sinf(halfAngle);

	// 回転軸を正規化
	Vector3 normalizedAxis = Math::Normalize(axis);

	// クォータニオンの成分を計算
	result.x = normalizedAxis.x * sinHalfAngle;
	result.y = normalizedAxis.y * sinHalfAngle;
	result.z = normalizedAxis.z * sinHalfAngle;
	result.w = cosf(halfAngle);

	return result;
}

Vector3 Quaternion::RotateVector(const Vector3& vector, const Quaternion& quaternion)
{
	// ベクトルをクォータニオン形式に変換
	Quaternion qVector = { vector.x, vector.y, vector.z, 0.0f };

	// クォータニオンの逆（共役）を取得
	Quaternion qConjugate = Conjugate(quaternion);

	// クォータニオンの掛け算を用いて回転を適用
	Quaternion qResult = Multiply(Multiply(quaternion, qVector), qConjugate);

	// 回転後のベクトルを取得
	return { qResult.x, qResult.y, qResult.z };
}

Matrix4x4 Quaternion::MakeRotateMatrix(const Quaternion& quaternion)
{
	Matrix4x4 result;

	float xx = quaternion.x * quaternion.x;
	float yy = quaternion.y * quaternion.y;
	float zz = quaternion.z * quaternion.z;
	float ww = quaternion.w * quaternion.w;
	float xy = quaternion.x * quaternion.y;
	float xz = quaternion.x * quaternion.z;
	float yz = quaternion.y * quaternion.z;
	float wx = quaternion.w * quaternion.x;
	float wy = quaternion.w * quaternion.y;
	float wz = quaternion.w * quaternion.z;

	result.m[0][0] = ww + xx - yy - zz;
	result.m[0][1] = 2.0f * (xy + wz);
	result.m[0][2] = 2.0f * (xz - wy);
	result.m[0][3] = 0.0f;

	result.m[1][0] = 2.0f * (xy - wz);
	result.m[1][1] = ww - xx + yy - zz;
	result.m[1][2] = 2.0f * (yz + wx);
	result.m[1][3] = 0.0f;

	result.m[2][0] = 2.0f * (xz + wy);
	result.m[2][1] = 2.0f * (yz - wx);
	result.m[2][2] = ww - xx - yy + zz;
	result.m[2][3] = 0.0f;

	result.m[3][0] = 0.0f;
	result.m[3][1] = 0.0f;
	result.m[3][2] = 0.0f;
	result.m[3][3] = 1.0f;

	return result;
}

Quaternion Quaternion::Slerp(const Quaternion& quaternion1, const Quaternion& quaternion2, float t)
{
	// 正規化された入力を想定
	Quaternion q1 = quaternion1;
	Quaternion q2 = quaternion2;

	// 内積を計算
	float dotProduct = Dot(Normalize(q1), Normalize(q2));

	// クォータニオンが逆向きの場合、q2を反転して最短経路を取る
	if (dotProduct < 0.0f)
	{
		dotProduct = -dotProduct;
		q2.w = -q2.w;
		q2.x = -q2.x;
		q2.y = -q2.y;
		q2.z = -q2.z;
	}

	// 内積が非常に大きい場合、線形補間を使用
	const float EPSILON = 1e-6f;
	if (dotProduct > 1.0f - EPSILON)
	{
		// 線形補間（Lerp）
		Quaternion result{};
		result.w = (1.0f - t) * q1.w + t * q2.w;
		result.x = (1.0f - t) * q1.x + t * q2.x;
		result.y = (1.0f - t) * q1.y + t * q2.y;
		result.z = (1.0f - t) * q1.z + t * q2.z;

		// 正規化
		Normalize(result);
	}

	// 球面線形補間（Slerp）
	float theta = std::acos(dotProduct); // θ = cos^-1(dotProduct)
	float sinTheta = std::sqrtf(1.0f - dotProduct * dotProduct); // sinθ

	float weight1 = std::sin((1.0f - t) * theta) / sinTheta;
	float weight2 = std::sin(t * theta) / sinTheta;

	Quaternion result{};
	result.w = weight1 * q1.w + weight2 * q2.w;
	result.x = weight1 * q1.x + weight2 * q2.x;
	result.y = weight1 * q1.y + weight2 * q2.y;
	result.z = weight1 * q1.z + weight2 * q2.z;

	return result;
}

void Quaternion::QuaternionScreenPrint(int x, int y, const Quaternion& quaternion, const char* label)
{
	// ラベルを出力
	Novice::ScreenPrintf(x, y, "%s", label);

	// クォータニオンを出力
	const float components[4] = { quaternion.x, quaternion.y, quaternion.z, quaternion.w };
	for (int i = 0; i < 4; i++)
	{
		Novice::ScreenPrintf(x + i * kColumnWidth, y + 20, "%6.02f", components[i]);
	}
}
