#pragma once
#include <corecrt_math.h>
#include <cmath>

/// <summary>
/// クォータニオン
/// </summary>
class Quaternion
{
public: /// ---------- メンバ変数 ---------- ///

	float x, y, z, w;

public: /// ---------- メンバ関数 ---------- ///

	// クォータニオンの積
	static Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs);

	// 単位クォータニオン
	static Quaternion IdentityQuaternion();

	// 共役クォータニオン
	static Quaternion Conjugate(const Quaternion& quaternion);

	// クォータニオンのNorm
	static float Norm(const Quaternion& quaternion);

	// 正規化したクォータニオン
	static Quaternion Normalize(const Quaternion& quaternion);

	// 逆クォータニオン
	static Quaternion Inverse(const Quaternion& quaternion);

	static void QuaternionScreenPrint(int x, int y, const Quaternion& quaternion, const char* label);

};

