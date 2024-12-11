#pragma once
#include "Vector3.h"
#include "Matrix4x4.h"
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

	// 任意軸回転を表すQuaternionの生成
	static Quaternion MakeRotateAxisAngleQuaternion(const Vector3& axis, float angle);
	
	// ベクトルをQuaternionで回転させた結果のベクトルを求める
	static Vector3 RotateVector(const Vector3& vector, const Quaternion& quaternion);
	
	// Quaternionから回転行列を求める
	static Matrix4x4 MakeRotateMatrix(const Quaternion& quaternion);

};

