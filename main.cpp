#include <Novice.h>
#include <imgui.h>
#include "Math//MathFunction.h"
#include <algorithm>

//間隔
static const int kRowHeight = 20;
static const int kColumnWidth = 60;

using namespace Math;

Matrix4x4 MakeCosThetaMatrix(const Vector3 cosTheta)
{
	return {
		cosTheta.x, 0.0f,     0.0f,     0.0f,
		0.0f,     cosTheta.y, 0.0f,     0.0f,
		0.0f,     0.0f,     cosTheta.z, 0.0f,
		0.0f,     0.0f,     0.0f,     1.0f
	};
}

Matrix4x4 MakeCrossProductMatrix(Vector3 normalizedAxis, float oneMinusCosTheta)
{
	return {
		normalizedAxis.x * normalizedAxis.x * oneMinusCosTheta, normalizedAxis.x * normalizedAxis.y * oneMinusCosTheta, normalizedAxis.x * normalizedAxis.z * oneMinusCosTheta, 0.0f,
		normalizedAxis.x * normalizedAxis.y * oneMinusCosTheta, normalizedAxis.y * normalizedAxis.y * oneMinusCosTheta, normalizedAxis.y * normalizedAxis.z * oneMinusCosTheta, 0.0f,
		normalizedAxis.x * normalizedAxis.z * oneMinusCosTheta, normalizedAxis.y * normalizedAxis.z * oneMinusCosTheta, normalizedAxis.z * normalizedAxis.z * oneMinusCosTheta, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	};
}

// 外積 n * n^t の項を作成
Matrix4x4 MakeOuterProductMatrix(Vector3 normalizedAxis, float sinTheta)
{
	return {
		0.0f, -normalizedAxis.z * sinTheta, normalizedAxis.y * sinTheta, 0.0f,
		normalizedAxis.z * sinTheta, 0.0f, -normalizedAxis.x * sinTheta, 0.0f,
		-normalizedAxis.y * sinTheta, normalizedAxis.x * sinTheta, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	};
}

Matrix4x4 MakeRotateAxisAngle(const Vector3& axis, float angle)
{
	// 回転軸を正規化
	Vector3 normalizedAxis = Normalize(axis);

	// cos(θ) と sin(θ) を計算
	Vector3 cosTheta = { cos(angle),cos(angle) ,cos(angle) };
	float sinTheta = -sin(angle);
	float oneMinusCosTheta = 1.0f - cos(angle);

	Matrix4x4 rotateMatrix;

	rotateMatrix = MakeCosThetaMatrix(cosTheta) + MakeCrossProductMatrix(normalizedAxis, oneMinusCosTheta) + MakeOuterProductMatrix(normalizedAxis, sinTheta);

	rotateMatrix.m[3][3] = 1.0f;

	return rotateMatrix;
}

// 行列のコメント
static void MatrixScreenPrint(int x, int y, Matrix4x4 matrix, const char* label)
{
	for (int row = 0; row < 4; row++)
	{
		for (int colum = 0; colum < 4; colum++)
		{
			Novice::ScreenPrintf(x + colum * kColumnWidth, y + row * kRowHeight + 20, "%6.03f", matrix.m[row][colum]);
		}
	}
	Novice::ScreenPrintf(x, y, "%s", label);
}

const char kWindowTitle[] = "学籍番号";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Vector3 axis = Normalize({ 1.0f, 1.0f, 1.0f });
	float angle = 0.44f;

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		Matrix4x4 rotateMatrix = MakeRotateAxisAngle(axis, angle);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		MatrixScreenPrint(0, 0, rotateMatrix, "rotateMatrix");

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
