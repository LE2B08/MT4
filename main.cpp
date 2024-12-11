#include <Novice.h>
#include <imgui.h>
#include "Math//MathFunction.h"
#include "Quaternion.h"
#include <algorithm>

using namespace Math;

static const int kWindowWidth = 1280;
static const int kWindowHeight = 720;

//間隔
static const int kRowHeight = 20;
static const int kColumnWidth = 60;

std::pair<Vector3, Vector3> ComputeCollisionvelocities(
	float mass1, const Vector3& velocity1, float mass2, const Vector3& velocity2, float coefficientOfRestitution, const Vector3& normal)
{
	// 衝突面の法線方向に速度を射影
	Vector3 project1 = Project(velocity1, normal); // 物体1の法線方向の速度
	Vector3 project2 = Project(velocity2, normal); // 物体2の法線方向の速度

	// 接線方向の速度成分（衝突後も変化しない）
	Vector3 sub1 = velocity1 - project1;
	Vector3 sub2 = velocity2 - project2;

	// 法線方向の衝突後の速度を計算
	Vector3 velocityAfter1 = (project1 * (mass1 - coefficientOfRestitution * mass2) +
		project2 * (1.0f + coefficientOfRestitution) * mass2) / (mass1 + mass2);

	Vector3 velocityAfter2 = (project2 * (mass2 - coefficientOfRestitution * mass1) +
		project1 * (1.0f + coefficientOfRestitution) * mass1) / (mass1 + mass2);

	// 最終的な速度：法線方向の速度 + 接線方向の速度
	return std::make_pair(velocityAfter1 + sub1, velocityAfter2 + sub2);
}

const char kWindowTitle[] = "学籍番号";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	int prevMouseX = 0;
	int prevMouseY = 0;
	bool isDragging = false;

	// 動いているかどうかのフラグ
	bool isActive = false;
	float deltaTime = 1.0f / 60.0f;

	Ball ball1{};
	ball1.position = { 0.0f, 0.0f, 0.0f };
	ball1.velocity = { 1.0f, 0.0f, 1.0f };
	ball1.mass = 2.0f;
	ball1.acceleration = { 0.0f, 0.0f, 0.0f };
	ball1.radius = 0.1f;
	ball1.color = RED;

	Ball ball2{};
	ball2.position = { 2.0f, 0.0f, 2.0f };
	ball2.velocity = {};
	ball2.mass = 2.0f;
	ball2.acceleration = { 0.0f, 0.0f, 0.0f };
	ball2.radius = 0.3f;
	ball2.color = WHITE;

	Sphere sphere1 = { .center{ball1.position.x, ball1.position.y, ball1.position.z}, .radius{ball1.radius} };
	Sphere sphere2 = { .center{ball2.position.x, ball2.position.y, ball2.position.z}, .radius{ball2.radius} };

	Vector3 translate{};
	Vector3 rotate{};

	Vector3 cameraTranslate = { 0.0f, 1.9f, -6.49f };
	Vector3 cameraRotate = { 0.26f, 0.0f, 0.0f };

	// ボールの速度
	Vector3 velocity1 = { 0.5f, 0.0f, 0.0f }; // Ball1の初期速度
	Vector3 velocity2 = { 0.0f, 0.0f, 0.0f }; // Ball2は最初静止

	float coefficientOfRestitution = 1.0f; // 反発係数（弾性の強さ）

	// 透視投影行列を作成
	Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, float(kWindowWidth) / float(kWindowHeight), 0.1f, 100.0f);
	// ViewportMatrixビューポート変換行列を作成
	Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, float(kWindowWidth), float(kWindowHeight), 0.0f, 1.0f);

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		// マウス入力を取得
		POINT mousePosition;
		GetCursorPos(&mousePosition);

		///
		/// ↓更新処理ここから
		///

		// マウスドラッグによる回転制御
		if (Novice::IsPressMouse(1))
		{
			if (!isDragging)
			{
				isDragging = true;
				prevMouseX = mousePosition.x;
				prevMouseY = mousePosition.y;
			}
			else
			{
				int deltaX = mousePosition.x - prevMouseX;
				int deltaY = mousePosition.y - prevMouseY;
				rotate.y += deltaX * 0.01f; // 水平方向の回転
				rotate.x += deltaY * 0.01f; // 垂直方向の回転
				prevMouseX = mousePosition.x;
				prevMouseY = mousePosition.y;
			}
		}
		else
		{
			isDragging = false;
		}

		// マウスホイールで前後移動
		int wheel = Novice::GetWheel();
		if (wheel != 0)
		{
			cameraTranslate.z += wheel * 0.01f; // ホイールの回転方向に応じて前後移動
		}

		// 球同士の距離を計算
		Vector3 normal = Normalize(ball1.position - ball2.position);

		ImGui::Begin("Control Window");
		if (ImGui::Button("Start"))
		{
			isActive = true; // 動きを開始
		}
		if (ImGui::Button("Reset"))
		{
			isActive = false; // 動きを停止
			ball1.position = { 0.0f, 0.0f, 0.0f }; // 初期位置にリセット
			ball2.position = { 2.0f, 0.0f, 2.0f };
			ball1.velocity = { 1.0f, 0.0f, 1.0f }; // 初期速度にリセット
			ball2.velocity = {};
			sphere1.center = { 0.0f, 0.0f, 0.0f };
			sphere2.center = { 2.0f, 0.0f, 2.0f };
		}

		ImGui::End();

		// ボールの動きがアクティブな場合のみ更新
		if (isActive) {
			// 速度に基づいてボールの位置を更新
			ball1.velocity += ball1.acceleration * deltaTime;
			ball2.velocity += ball2.acceleration * deltaTime;

			ball1.position += ball1.velocity * deltaTime;
			ball2.position += ball2.velocity * deltaTime;

			// 対応するスフィアの中心も更新
			sphere1.center += ball1.velocity * deltaTime;
			sphere2.center += ball2.velocity * deltaTime;

			// 衝突判定
			float distance = Length(ball1.position - ball2.position);
			if (distance <= ball1.radius + ball2.radius) {
				// 衝突後の新しい速度を計算
				auto [newVelocity1, newVelocity2] = ComputeCollisionvelocities(
					ball1.mass, ball1.velocity, ball2.mass, ball2.velocity, coefficientOfRestitution, Normalize(ball1.position - ball2.position)
				);

				// 計算した速度でボールの速度を更新
				ball1.velocity = newVelocity1;
				ball2.velocity = newVelocity2;

				// 衝突後にボールが重ならないように、位置補正を行う
				Vector3 correction = Normalize(ball1.position - ball2.position) * (ball1.radius + ball2.radius - distance) * 0.5f;
				ball1.position += correction;
				ball2.position -= correction;

				// スフィアの位置もボールの位置に合わせて補正
				sphere1.center = ball1.position;
				sphere2.center = ball2.position;
			}
		}

		// 各種行列の計算
		Matrix4x4 worldMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, rotate, translate);
		Matrix4x4 cameraMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, cameraRotate, cameraTranslate);
		Matrix4x4 viewWorldMatrix = Inverse(worldMatrix);
		Matrix4x4 viewCameraMatrix = Inverse(cameraMatrix);
		Matrix4x4 viewProjectionMatrix = Multiply(viewWorldMatrix, Multiply(viewCameraMatrix, projectionMatrix));

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		DrawGrid(viewProjectionMatrix, viewportMatrix);
		DrawSphere(sphere1, viewProjectionMatrix, viewportMatrix, ball1.color);
		DrawSphere(sphere2, viewProjectionMatrix, viewportMatrix, ball2.color);

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
