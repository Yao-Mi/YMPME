/* 可选 GRE 适配；物理核心不包含任何 GRE 头文件。 */
#ifndef YMPME_GRE_H
#define YMPME_GRE_H
#include "../OPOBJ/YMPME_Body.h"
#include "YMGRE/OPOBJ/YMGRE_OBJ.h"
#ifdef __cplusplus
extern "C" {
#endif
/** 从不可变局部顶点重建 GRE 世界顶点，避免累加 LocalToWorld。
 * localVertices 由应用持有，必须与 object->pointList 完全不重叠，数量等于 pointNum。
 * 局部网格使用 GRE 单位；object->scale 只缩放网格，unitsPerMeter 只换算物理位置。
 * alpha=1 为当前物理位置；不转换坐标轴、不更新朝向、不改变碰撞体尺寸。
 * 写入 pointList、已有高级顶点 base、WorldCoordinate 和局部 AABB（GRE 剔除阶段会再乘 scale）。
 * object 和所有顶点内存均由应用管理，调用期间不得被其他渲染线程读取。
 */
pme_result YMPME_GRE_SyncObject(const pme_body* body, GRE_Object4d object,
	const gre_vertex4d* localVertices, size_t count, float unitsPerMeter, float alpha);
#ifdef __cplusplus
}
#endif
#endif
