#pragma once
#include "Vector3.h"
#include "Matrix4x4.h"

struct Quaternion
{
	float x, y, z, w;
};

// Quaternionの積
Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs);
// 単位Quaternionを返す
Quaternion IdentityQuaternion();
// 共役Quaternionを返す
Quaternion Conjugate(const Quaternion& quaternion);
// QuaternionのNormを返す
float Norm(const Quaternion& quaternion);
// 正規化したQuaternionを返す
Quaternion Normalize(const Quaternion& quaternion);
// 逆Quaternionを返す
Quaternion Inverse(const Quaternion& quaternion);
// QuaternionScreenPrintf
void QuaternionScreenPrintf(int x, int y, const Quaternion& quaternion, const char* label);
// 任意軸回転を表すQuaternionの生成
Quaternion MakeRotateAxisAngleQuaternion(const Vector3& axis, float angle);
// ベクトルをQuaternionで回転させた結果のベクトルを求める
Vector3 RotateVector(const Vector3& vector, const Quaternion& quaternion);
// Quaternionから回転行列を求める
Matrix4x4 MakeRotateMatrix(const Quaternion& quaternion);
