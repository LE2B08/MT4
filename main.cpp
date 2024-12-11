#include <Novice.h>
#include <imgui.h>
#include "Math//MathFunction.h"
#include "Quaternion.h"
#include <algorithm>

static const int kRowHeight = 20;

using namespace Math;

const char kWindowTitle[] = "学籍番号";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Quaternion q1 = { 2.0f,3.0f,4.0f,1.0f };
	Quaternion q2 = { 1.0f,3.0f,5.0f,2.0f };
	Quaternion identity = Quaternion::IdentityQuaternion();
	Quaternion conj = Quaternion::Conjugate(q1);
	Quaternion inv = Quaternion::Inverse(q1);
	Quaternion normal = Quaternion::Normalize(q1);
	Quaternion mul1 = Quaternion::Multiply(q1, q2);
	Quaternion mul2 = Quaternion::Multiply(q2, q1);
	float norm = Quaternion::Norm(q1);

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

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		Quaternion::QuaternionScreenPrint(0, kRowHeight * 0, identity, "Identity");
		Quaternion::QuaternionScreenPrint(0, kRowHeight * 3, conj, "Conjugate");
		Quaternion::QuaternionScreenPrint(0, kRowHeight * 6, inv, "Inverse");
		Quaternion::QuaternionScreenPrint(0, kRowHeight * 9, normal, "Normalize");
		Quaternion::QuaternionScreenPrint(0, kRowHeight * 12, mul1, "Multiply(q1, q2)");
		Quaternion::QuaternionScreenPrint(0, kRowHeight * 15, mul2, "Multiply(q2, q1)");
		Novice::ScreenPrintf(0, kRowHeight * 18, "Norm : %.2f", norm);

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
