#pragma once
#include "Vector3.h"

/// <summary>
/// ボールの構造体
/// </summary>
struct Ball final
{
	Vector3 position;           // ボールの位置
	Vector3 velocity;           // ボールの速度
	Vector3 acceleration;       // ボールの加速度
	float mass;                 // ボールの質量
	float radius;               // ボールの半径
	unsigned int color;         // ボールの色
};