#include "YMPME_Drive.h"
#include <string.h>
static float approach(float value, float target, float change)
{
	return value + YMPME_Clamp(target - value, -change, change);
}
void YMPME_Drive_Reset(pme_drive *c)
{
	if (!c)
		return;
	memset(c, 0, sizeof(*c));
	c->track = 1.04f;
	c->radius = 0.26f;
	c->casterRadius = 0.13f;
	c->casterTrail = 0.14f;
	c->casterZ = -0.57f;
	c->acceleration = 4;
}
pme_result YMPME_Drive_Step(pme_drive *c, float left, float right, float dt)
{
	float speed, omega, angle, distance, sinc, lateral, casterRate;
	if (!c || !isfinite(left) || !isfinite(right) || !isfinite(dt) || dt <= 0 || dt > 0.05f ||
		!isfinite(c->track) || c->track <= 0 || !isfinite(c->radius) || c->radius <= 0 ||
		!isfinite(c->casterTrail) || c->casterTrail <= 0 || !isfinite(c->casterRadius) ||
		c->casterRadius <= 0 || !isfinite(c->acceleration) || c->acceleration < 0)
		return PME_INVALID_ARGUMENT;
	c->left = approach(c->left, left, c->acceleration * dt);
	c->right = approach(c->right, right, c->acceleration * dt);
	speed = (c->left + c->right) * 0.5f;
	omega = (c->left - c->right) / c->track;
	angle = omega * dt;
	/* 在本步轮速恒定的假设下按圆弧精确积分，避免原地转向发生平移。 */
	sinc = fabsf(angle) < 1e-5f ? 1.0f : sinf(angle * 0.5f) / (angle * 0.5f);
	distance = speed * dt * sinc;
	c->x += distance * sinf(c->yaw + angle * 0.5f);
	c->z += distance * cosf(c->yaw + angle * 0.5f);
	c->yaw = remainderf(c->yaw + angle, 6.28318530718f);
	c->leftAngle = remainderf(c->leftAngle + c->left * dt / c->radius, 6.28318530718f);
	c->rightAngle = remainderf(c->rightAngle + c->right * dt / c->radius, 6.28318530718f);
	/* 拖距脚轮：枢轴横向速度 = omega*casterZ；轮平面沿接触速度被动摆动。 */
	lateral = omega * c->casterZ;
	if (speed < -0.01f && fabsf(c->casterAngle) < 1e-5f && fabsf(omega) < 1e-5f)
		c->casterAngle = 0.015f; /* 打破倒车正对齐的不稳定对称位置。 */
	casterRate = (lateral * cosf(c->casterAngle) - speed * sinf(c->casterAngle)) / c->casterTrail - omega;
	c->casterAngle = remainderf(c->casterAngle + YMPME_Clamp(casterRate, -30, 30) * dt, 6.28318530718f);
	c->casterSpin =
		remainderf(c->casterSpin +
					   (lateral * sinf(c->casterAngle) + speed * cosf(c->casterAngle)) * dt / c->casterRadius,
				   6.28318530718f);
	return PME_OK;
}
