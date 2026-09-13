#ifndef YMPME_ROPEGRE_H
#define YMPME_ROPEGRE_H
#include "../MODEL/ROPE/YMPME_Rope.h"
#include "YMGRE/OPOBJ/YMGRE_OBJ.h"
#ifdef __cplusplus
extern "C"
{
#endif
	typedef struct pme_rope_gre_
	{
		const pme_rope *rope;
		const pme_rope_node *nodes;
		GRE_Object4d object;
		size_t count, linkCount;
		int closed;
		float unitsPerMeter, radius, alpha;
		uint64_t revision, updates;
		size_t bytesWritten;
	} pme_rope_gre;
	/** 初始化时创建一份持久方形截面绳网格，radius 是截面半宽(m)。
	 * binding 不得正在持有网格；成功后 binding 拥有网格，Destroy 释放。
	 * 不复制模型节点；rope 与其缓冲必须比 binding 长寿。
	 * 模型重新 Init 后同拓扑可 Rebind；节点数/闭合状态改变须 Destroy 后 Create。
	 * 失败时 binding 不变；GRE 分配只发生于 Create/Destroy。
	 */
	pme_result YMPME_GRE_Rope_Create(pme_rope_gre *binding, const pme_rope *rope, float unitsPerMeter,
									 float radius, GRErgb24 color);
	/** 直接借用节点状态，插值后一次写入持久网格；相同 revision/alpha 跳过。
	 * alpha [0,1]；不会修改模型。禁止和模型写入/网格渲染无保护并发。
	 * bytesWritten 是本次顶点位置、法线及边界写入的字节数（缓存命中为0）。
	 * 不生成状态副本，不分配堆内存。此绑定使用普通顶点/平面法线，不提供平滑着色。
	 */
	pme_result YMPME_GRE_Rope_Sync(pme_rope_gre *binding, float alpha);
	/** 同拓扑重新绑定模型并使缓存失效，复用原网格，无分配。 */
	pme_result YMPME_GRE_Rope_Rebind(pme_rope_gre *binding, const pme_rope *rope);
	void YMPME_GRE_Rope_Destroy(pme_rope_gre *binding);
#ifdef __cplusplus
}
#endif
#endif
