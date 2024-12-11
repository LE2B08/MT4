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

	static float Dot(const Quaternion& q1, const Quaternion& q2);

	// 逆クォータニオン
	static Quaternion Inverse(const Quaternion& quaternion);

	static void QuaternionScreenPrint(int x, int y, const Quaternion& quaternion, const char* label);

	// 任意軸回転を表すQuaternionの生成
	static Quaternion MakeRotateAxisAngleQuaternion(const Vector3& axis, float angle);
	
	// ベクトルをQuaternionで回転させた結果のベクトルを求める
	static Vector3 RotateVector(const Vector3& vector, const Quaternion& quaternion);
	
	// Quaternionから回転行列を求める
	static Matrix4x4 MakeRotateMatrix(const Quaternion& quaternion);

	// 球面線形補間
	static Quaternion Slerp(const Quaternion& q0, const Quaternion& q1, float t);

public: /// ---------- オペレーター演算子 ---------- ///
	Quaternion() : w(1), x(0), y(0), z(0) {}
	Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {} 

	Quaternion operator-() const{ return Quaternion(-w, -x, -y, -z); } 
	Quaternion operator+(const Quaternion& q) const { return Quaternion(w + q.w, x + q.x, y + q.y, z + q.z); }
	Quaternion operator-(const Quaternion& q) const { return Quaternion(w - q.w, x - q.x, y - q.y, z - q.z); } 
	Quaternion operator*(float scalar) const { return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar); }
	friend Quaternion operator*(float scalar, const Quaternion& q) { return q * scalar; }
};

