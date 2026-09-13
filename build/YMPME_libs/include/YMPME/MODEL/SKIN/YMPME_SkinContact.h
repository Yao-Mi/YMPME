#ifndef YMPME_SKIN_CONTACT_H
#define YMPME_SKIN_CONTACT_H
#include "../../CORE/YMPME_Surface.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_skin_contact_desc_
	{
		float stiffness, damping, friction, tangentDamping, maxPressure;
	} pme_skin_contact_desc;
	typedef struct pme_skin_probe_
	{
		pme_vec3 center, velocity;
		float radius;
	} pme_skin_probe;
	typedef struct pme_skin_contact_result_
	{
		pme_wrench probeWrench;
		size_t contacts;
		float maxPenetration;
		int saturated;
	} pme_skin_contact_result;
	pme_skin_contact_desc YMPME_SkinContact_DefaultDesc(void);
	/* 表面采样点弹性地基近似：球内且朝向球心的patch生成压缩载荷。
	 * stiffness Pa/m，damping Pa*s/m；切向黏性摩擦受mu*法向力限制。
	 * pressure上限显式置saturated，不是精确三角形碰撞/FEM/CCD。
	 * forceOnSkin按patch顺序，probeWrench为反作用、力矩关于球心。
	 * 输入输出不得重叠；全部预检后写，失败不改变输出，不分配内存。 */
	pme_result YMPME_SkinContact_Sphere(const pme_skin_contact_desc *desc,
										const pme_surface_patch *patches, size_t count,
										const pme_skin_probe *probe, pme_vec3 *forceOnSkin,
										size_t capacity, pme_skin_contact_result *out);
	/* 自由转动球：径向接触法线；omega为世界rad/s，用v+omega×r计算摩擦。
	 * 仍是表面点采样近似（不做三角面/连续碰撞）。输出力矩关于球心。
	 * 原Sphere的皮肤法线按压语义及所有结构布局不变。 */
	pme_result YMPME_SkinContact_SphereRigid(const pme_skin_contact_desc *desc,
											 const pme_surface_patch *patches, size_t count,
											 const pme_skin_probe *probe, pme_vec3 angularVelocity,
											 pme_vec3 *forceOnSkin, size_t capacity,
											 pme_skin_contact_result *out);
#ifdef __cplusplus
}
#endif
#endif
