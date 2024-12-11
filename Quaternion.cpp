#include "Quaternion.h"
#include "Novice.h"

//間隔
static const int kRowHeight = 20;
static const int kColumnWidth = 60;

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

void Quaternion::QuaternionScreenPrint(int x, int y, const Quaternion& quaternion, const char* label)
{
	// ラベルを出力
	Novice::ScreenPrintf(x, y, "%s", label);

	// クォータニオンを出力
	const float components[4] = { quaternion.x, quaternion.y, quaternion.z, quaternion.w };
	for (int i = 0; i < 4; i++)
	{
		Novice::ScreenPrintf(x + i * kColumnWidth, y + 20, "%6.03f", components[i]);
	}
}
