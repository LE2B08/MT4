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

	Quaternion rotation0 = Quaternion::MakeRotateAxisAngleQuaternion({ 0.71f, 0.71f, 0.0f }, 0.3f);
	Quaternion rotation1 = Quaternion::MakeRotateAxisAngleQuaternion({ 0.71f, 0.0f, 0.71f }, 3.141592f);

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

		Quaternion interpolate0 = Quaternion::Slerp(rotation0, rotation1, 0.0f);
		Quaternion interpolate1 = Quaternion::Slerp(rotation0, rotation1, 0.3f);
		Quaternion interpolate2 = Quaternion::Slerp(rotation0, rotation1, 0.5f);
		Quaternion interpolate3 = Quaternion::Slerp(rotation0, rotation1, 0.7f);
		Quaternion interpolate4 = Quaternion::Slerp(rotation0, rotation1, 1.0f);


		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		Quaternion::QuaternionScreenPrint(0, 0, interpolate0, "interpolate0, Slerp(q0, q1, 0.0f) ");
		Quaternion::QuaternionScreenPrint(0, kRowHeight * 3, interpolate1, "interpolate1, Slerp(q0, q1, 0.3f) ");
		Quaternion::QuaternionScreenPrint(0, kRowHeight * 6, interpolate2, "interpolate2, Slerp(q0, q1, 0.5f) ");
		Quaternion::QuaternionScreenPrint(0, kRowHeight * 9, interpolate3, "interpolate3, Slerp(q0, q1, 0.7f) ");
		Quaternion::QuaternionScreenPrint(0, kRowHeight * 12, interpolate4, "interpolate4, Slerp(q0, q1, 1.0f) ");

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
