/* 刚体状态及碰撞形状；首版只求解平移，不含角速度。 */
#ifndef YMPME_OBJ_H
#define YMPME_OBJ_H
#include "../CONFIG/YMPME_PubType.h"
#include "../CORE/YMPME_ConvexMesh.h"
typedef enum pme_body_type_
{
	PME_STATIC,    /* 不积分、不被碰撞推动。 */
	PME_DYNAMIC,   /* 受力且参与碰撞响应。 */
	PME_KINEMATIC  /* 按指定速度运动，不被碰撞推动。 */
} pme_body_type;
typedef enum pme_shape_type_ { PME_SPHERE, PME_BOX, PME_CONVEX_MESH } pme_shape_type;
typedef struct pme_body_desc_
{
	pme_body_type type;
	pme_shape_type shape;
	pme_vec3 position, velocity;
	const pme_convex_mesh* mesh; /* PME_CONVEX_MESH 借用的闭合凸网格。 */
	pme_rotation rotation;      /* 仅 Mesh 使用；正交旋转，无隐含缩放。 */
	uint32_t collisionGroup;   /* 0..31，世界配对矩阵的组号。 */
	uint32_t ignoreAssembly;   /* 相同且非零时永不检测，适合同一装配体内部。 */
	pme_vec3 halfExtents; /* 盒体半尺寸；不随渲染缩放变化。 */
	float radius;
	float mass;          /* Dynamic 必须 > 0；其他类型忽略。 */
	float restitution;   /* [0,1]；碰撞取双方较小值。 */
	float friction;      /* >= 0；碰撞取几何平均值。 */
	float linearDamping; /* >= 0，单位 1/s。 */
	uint32_t category, mask; /* 双向位掩码过滤。 */
	int sensor;          /* 检测重叠但不产生响应。 */
	void* userData;       /* 借用的应用对象，库不释放。 */
} pme_body_desc;
typedef struct pme_body_
{
	pme_body_desc state;
	pme_vec3 previousPosition;
	pme_vec3 force;
	float inverseMass;
	int active;
} pme_body;
typedef pme_body* PME_Body;
typedef struct pme_contact_
{
	pme_vec3 normal; /* A 指向 B；A 沿 -normal、B 沿 +normal 分离。 */
	float penetration; /* 接触边界也算碰撞，此时深度为 0。 */
} pme_contact;
#endif
