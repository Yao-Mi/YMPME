/* 独立绳索模型：仅物理数据，不依赖渲染器。所有缓冲由调用方持有。 */
#ifndef YMPME_ROPE_H
#define YMPME_ROPE_H
#include "../../CORE/YMPME_MathBase.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_rope_node_
	{
		pme_vec3 position, previous, velocity;
		float inverseMass;
		/* 求解工作区，调用方不得直接改写。 */
		pme_vec3 old, target;
	} pme_rope_node;
	typedef struct pme_rope_link_
	{
		float restLength;
		double lambda, tension;
	} pme_rope_link;
	typedef struct pme_rope_desc_
	{
		size_t count;
		int closed;
		const pme_vec3 *positions;
		const float *masses;	  /* kg；0 表示固定节点。 */
		const float *restLengths; /* m；NULL 使用初始相邻距离。闭环含末节点到首节点。 */
		pme_vec3 gravity;
		float damping, compliance, maxSubstep; /* 1/s、m/N、s。compliance=0 为不可伸长约束。 */
		unsigned iterations;
	} pme_rope_desc;
	typedef struct pme_rope_
	{
		pme_rope_node *nodes;
		pme_rope_link *links;
		size_t count, linkCount;
		int closed;
		pme_vec3 gravity;
		float damping, compliance, maxSubstep;
		unsigned iterations;
		uint64_t revision;
		double time;
	} pme_rope;
	typedef pme_rope *PME_Rope;
	typedef struct pme_rope_view_
	{
		const pme_rope_node *nodes;
		const pme_rope_link *links;
		size_t count, linkCount;
		int closed;
		uint64_t revision;
	} pme_rope_view;
	/** 默认重力 (0,-9.81,0)，阻尼 .05/s，24 次迭代，最大子步 1/600s。 */
	pme_rope_desc YMPME_Rope_DefaultDesc(void);
	/** 节点 2..4096（闭环至少 3），质量 0 或 [1e-6,1e6]kg，长度 [1e-5,1e4]m。
	 * 有限坐标绝对值 <=1e5m；输入数组与 rope/输出缓存不得重叠。
	 * 容量不足或描述非法时所有输出不变。输入数组仅初始化期间借用；
	 * nodes/links 持续借用直到最后一次使用，禁止直接改写或浅拷贝后独立步进。
	 * 只有拉伸约束，无抗弯、碰撞、自碰撞或缠绕。 */
	pme_result YMPME_Rope_Init(PME_Rope rope, const pme_rope_desc *desc, pme_rope_node *nodes,
							   size_t nodeCapacity, pme_rope_link *links, size_t linkCapacity);
	/** 开放链预设：初始点均匀分布在 start/end 连线上，restLength 总和为 length。
	 * length 不得小于两端距离。每节点质量 nodeMass，按参数固定端点；
	 * 绳长多于跨度时从直线零初速自然下垂。端点重物可随后 SetMass。
	 * 采用 DefaultDesc。与通用 Init 相同的缓冲/失败约定。 */
	pme_result YMPME_Rope_InitSpan(PME_Rope rope, pme_rope_node *nodes, size_t nodeCapacity,
								   pme_rope_link *links, size_t linkCapacity, size_t count, pme_vec3 start,
								   pme_vec3 end, float length, float nodeMass, int pinStart, int pinEnd);
	/** 外部每步 dt=[1e-6,.05]s，内部细分。previous 保存本次外部步进前位置。
	 * 只由调用方推进，不自动注册 World。Step 不分配内存。
	 * 非法 dt 或求解参数不改状态；gravity/damping/compliance/maxSubstep/iterations
	 * 可在两步间修改，范围同描述。调用方须保持已初始化结构与缓冲有效。 */
	pme_result YMPME_Rope_Step(PME_Rope rope, float dt);
	/** 仅对固定节点设置目标位置；下一次 Step 内插移动到目标，保持锚点速度连续于该步。
	 * 这是规定运动端点，不向外部刚体自动反馈力；可读取相邻 link.tension。 */
	pme_result YMPME_Rope_SetAnchor(PME_Rope rope, size_t index, pme_vec3 target);
	/** mass=0 固定当前节点，正质量解除固定；不改位置，切换固定状态时清零速度。 */
	pme_result YMPME_Rope_SetMass(PME_Rope rope, size_t index, float mass);
	/** 冲量单位 N*s，只能作用于动态节点；不会立即改变位置。 */
	pme_result YMPME_Rope_AddImpulse(PME_Rope rope, size_t index, pme_vec3 impulse);
	/** O(1) 借用只读视图，无分配/状态拷贝。节点数和拓扑在实例生命周期内不变。
	 * revision 标记状态变化；重新 Init 后绑定须重新建立，不跨实例比较 revision。
	 * 禁止在 Step/改参/重新 Init 与读取视图之间无保护并发。 */
	pme_rope_view YMPME_Rope_GetView(const pme_rope *rope);
#ifdef __cplusplus
}
#endif
#endif
