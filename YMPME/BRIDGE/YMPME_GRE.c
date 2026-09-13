#include "YMPME_GRE.h"
#include "../CORE/YMPME_MathBase.h"
pme_result YMPME_GRE_SyncObject(const pme_body* body, GRE_Object4d object,
	const gre_vertex4d* local, size_t count, float unitsPerMeter, float alpha)
{
	size_t i;
	pme_vec3 p;
	if (!body || !body->active || !object || !local || !object->pointList ||
		object->pointNum <= 0 || count != (size_t)object->pointNum ||
		local == object->pointList || !isfinite(unitsPerMeter) || unitsPerMeter <= 0 ||
		!isfinite(object->scale) || object->scale <= 0 || !isfinite(alpha)) return PME_INVALID_ARGUMENT;
	p = YMPME_VecScale(YMPME_Body_InterpolatedPosition(body, alpha), unitsPerMeter);
	object->WorldCoordinate = (gre_fvector4d){p.x, p.y, p.z, 1};
	for (i = 0; i < count; ++i)
	{
		gre_vertex4d vertex = local[i];
		vertex.pos.x = vertex.pos.x*object->scale+p.x;
		vertex.pos.y = vertex.pos.y*object->scale+p.y;
		vertex.pos.z = vertex.pos.z*object->scale+p.z;
		vertex.pos.w = 1;
		object->pointList[i] = vertex;
		if (object->pointList_wN) object->pointList_wN[i].base = vertex;
		if (object->pointList_wN_) object->pointList_wN_[i].base = vertex;
		if (i == 0) object->BoundingBoxMin = object->BoundingBoxMax = local[i].pos;
		else
		{
			object->BoundingBoxMin.x = fminf(object->BoundingBoxMin.x, local[i].pos.x);
			object->BoundingBoxMin.y = fminf(object->BoundingBoxMin.y, local[i].pos.y);
			object->BoundingBoxMin.z = fminf(object->BoundingBoxMin.z, local[i].pos.z);
			object->BoundingBoxMax.x = fmaxf(object->BoundingBoxMax.x, local[i].pos.x);
			object->BoundingBoxMax.y = fmaxf(object->BoundingBoxMax.y, local[i].pos.y);
			object->BoundingBoxMax.z = fmaxf(object->BoundingBoxMax.z, local[i].pos.z);
		}
	}
	return PME_OK;
}
