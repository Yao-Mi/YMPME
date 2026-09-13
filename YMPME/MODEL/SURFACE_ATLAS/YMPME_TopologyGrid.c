#include "YMPME_TopologyGrid.h"
#include <string.h>
pme_topology_grid_desc YMPME_TopologyGrid_DefaultDesc(void)
{
	return (pme_topology_grid_desc){64, 64, 4000, 80, 0x504d4531u};
}
static uint32_t random32(uint32_t *s)
{
	*s = *s * 1664525u + 1013904223u;
	return *s;
}
static void orthogonalize(float *xy, size_t n)
{
	for (unsigned axis = 0; axis < 2; axis++)
	{
		double mean = 0, norm = 0, projection = 0;
		for (size_t i = 0; i < n; i++)
			mean += xy[i * 2 + axis];
		mean /= n;
		for (size_t i = 0; i < n; i++)
			xy[i * 2 + axis] -= (float)mean;
		if (axis)
		{
			for (size_t i = 0; i < n; i++)
				projection += (double)xy[i * 2] * xy[i * 2 + 1];
			for (size_t i = 0; i < n; i++)
				xy[i * 2 + 1] -= (float)projection * xy[i * 2];
		}
		for (size_t i = 0; i < n; i++)
			norm += (double)xy[i * 2 + axis] * xy[i * 2 + axis];
		if (norm > 1e-30)
		{
			float inv = (float)(1 / sqrt(norm));
			for (size_t i = 0; i < n; i++)
				xy[i * 2 + axis] *= inv;
		}
	}
}
static int less(uint32_t a, uint32_t b, const float *xy, unsigned axis)
{
	float x = xy[a * 2 + axis], y = xy[b * 2 + axis];
	return x != y ? x < y : a < b;
}
static void sift(uint32_t *order, size_t root, size_t n, const float *xy, unsigned axis)
{
	while (root < n / 2)
	{
		size_t child = root * 2 + 1;
		if (child + 1 < n && less(order[child], order[child + 1], xy, axis))
			child++;
		if (!less(order[root], order[child], xy, axis))
			break;
		uint32_t t = order[root];
		order[root] = order[child];
		order[child] = t;
		root = child;
	}
}
static void partition(uint32_t *order, size_t n, const float *xy, unsigned x, unsigned y,
					  unsigned width, unsigned height, unsigned stride, uint32_t *cells,
					  uint32_t *faces)
{
	if (!n)
		return;
	if (width * height == 1)
	{
		cells[y * stride + x] = order[0];
		faces[order[0]] = y * stride + x;
		return;
	}
	unsigned axis = width >= height ? 0 : 1;
	for (size_t i = n / 2; i > 0; i--)
		sift(order, i - 1, n, xy, axis);
	for (size_t i = n; i > 1; i--)
	{
		uint32_t t = order[0];
		order[0] = order[i - 1];
		order[i - 1] = t;
		sift(order, 0, i - 1, xy, axis);
	}
	unsigned a = axis ? height / 2 : width / 2, b = axis ? height : width;
	size_t left = n * a / b;
	if (axis)
	{
		partition(order, left, xy, x, y, width, a, stride, cells, faces);
		partition(order + left, n - left, xy, x, y + a, width, height - a, stride, cells, faces);
	}
	else
	{
		partition(order, left, xy, x, y, a, height, stride, cells, faces);
		partition(order + left, n - left, xy, x + a, y, width - a, height, stride, cells, faces);
	}
}
static double distance2(uint32_t a, uint32_t b, unsigned width)
{
	double x = (double)(a % width) - (b % width), y = (double)(a / width) - (b / width);
	return x * x + y * y;
}
static double swapDelta(uint32_t p, uint32_t q, uint32_t pc, uint32_t qc, const int32_t *adj,
						const uint32_t *faceCell, unsigned width)
{
	double delta = 0;
	for (int k = 0; k < 3; k++)
	{
		int32_t j = adj[p * 3 + k];
		if (j >= 0 && (uint32_t)j != q)
			delta += distance2(qc, faceCell[j], width) - distance2(pc, faceCell[j], width);
		if (q == PME_TOPOLOGY_EMPTY)
			continue;
		j = adj[q * 3 + k];
		if (j >= 0 && (uint32_t)j != p)
			delta += distance2(pc, faceCell[j], width) - distance2(qc, faceCell[j], width);
	}
	return delta;
}
static void measure(const int32_t *adj, size_t n, const uint32_t *faces, unsigned width,
					pme_topology_grid_result *r)
{
	double total = 0;
	size_t close = 0;
	r->edges = 0;
	r->maxDistance = 0;
	for (size_t i = 0; i < n; i++)
		for (int k = 0; k < 3; k++)
			if (adj[i * 3 + k] > (int32_t)i)
			{
				float d = (float)sqrt(distance2(faces[i], faces[adj[i * 3 + k]], width));
				total += d;
				r->edges++;
				close += d <= 2.00001f;
				r->maxDistance = fmaxf(r->maxDistance, d);
			}
	r->meanDistance = r->edges ? (float)(total / r->edges) : 0;
	r->withinTwoFraction = r->edges ? (float)close / r->edges : 1;
}
pme_result YMPME_TopologyGrid_Build(const pme_topology_grid_desc *d, const int32_t *adj, size_t n,
									pme_topology_grid_workspace *w, uint32_t *cells,
									size_t cellCapacity, uint32_t *faces, size_t faceCapacity,
									pme_topology_grid_result *out)
{
	if (!d || !adj || !w || !out || !cells || !faces || !w->xy || !w->next || !w->order || !n ||
		n > INT32_MAX / 3 || !d->width || !d->height || d->width > 4096 || d->height > 4096 ||
		d->diffusionSteps > 10000 || d->swapPasses > 10000)
		return PME_INVALID_ARGUMENT;
	size_t count = (size_t)d->width * d->height;
	if (n > count || cellCapacity < count || faceCapacity < n || w->faceCapacity < n)
		return PME_CAPACITY_EXCEEDED;
	for (size_t i = 0; i < n; i++)
		for (int k = 0; k < 3; k++)
		{
			int32_t j = adj[i * 3 + k];
			if (j < -1 || j == (int32_t)i || (j >= 0 && (size_t)j >= n))
				return PME_INVALID_ARGUMENT;
			if (j < 0)
				continue;
			int reciprocal = 0;
			for (int l = 0; l < 3; l++)
			{
				reciprocal += adj[j * 3 + l] == (int32_t)i;
				if (l < k && adj[i * 3 + l] == j)
					return PME_INVALID_ARGUMENT;
			}
			if (reciprocal != 1)
				return PME_INVALID_ARGUMENT;
		}
	uint32_t seed = d->seed;
	for (size_t i = 0; i < n; i++)
	{
		w->order[i] = (uint32_t)i;
		for (int a = 0; a < 2; a++)
			w->xy[i * 2 + a] = (random32(&seed) >> 8) / 16777216.f - .5f;
	}
	orthogonalize(w->xy, n);
	for (uint32_t step = 0; step < d->diffusionSteps; step++)
	{
		for (size_t i = 0; i < n; i++)
			for (int a = 0; a < 2; a++)
			{
				float v = w->xy[i * 2 + a], delta = 0;
				for (int k = 0; k < 3; k++)
				{
					int32_t j = adj[i * 3 + k];
					if (j >= 0)
						delta += w->xy[j * 2 + a] - v;
				}
				w->next[i * 2 + a] = v + .2f * delta;
			}
		orthogonalize(w->next, n);
		memcpy(w->xy, w->next, n * 2 * sizeof(float));
	}
	for (size_t i = 0; i < count; i++)
		cells[i] = PME_TOPOLOGY_EMPTY;
	partition(w->order, n, w->xy, 0, 0, d->width, d->height, d->width, cells, faces);
	pme_topology_grid_result r = {0};
	r.faces = n;
	r.cells = count;
	measure(adj, n, faces, d->width, &r);
	r.initialMeanDistance = r.meanDistance;
	for (uint32_t pass = 0; pass < d->swapPasses; pass++)
	{
		size_t moved = 0;
		for (size_t t = 0; t < n; t++)
		{
			uint32_t p = w->order[t], pc = faces[p], best = pc;
			double gain = 0;
			float cx = 0, cy = 0;
			unsigned degree = 0;
			for (int k = 0; k < 3; k++)
			{
				int32_t j = adj[p * 3 + k];
				if (j >= 0)
				{
					cx += faces[j] % d->width;
					cy += faces[j] / d->width;
					degree++;
				}
			}
			if (!degree)
				continue;
			int x = (int)lroundf(cx / degree), y = (int)lroundf(cy / degree);
			/* Local targets plus global exchanges help escape ordering artefacts. */
			for (int trial = 0; trial < 35; trial++)
			{
				uint32_t qc;
				if (trial < 25)
				{
					int xx = x + trial % 5 - 2, yy = y + trial / 5 - 2;
					if (xx < 0 || yy < 0 || xx >= (int)d->width || yy >= (int)d->height)
						continue;
					qc = (uint32_t)yy * d->width + (uint32_t)xx;
				}
				else
					qc = random32(&seed) % (uint32_t)count;
				if (qc == pc)
					continue;
				double delta = swapDelta(p, cells[qc], pc, qc, adj, faces, d->width);
				if (delta < gain)
				{
					gain = delta;
					best = qc;
				}
			}
			if (best != pc)
			{
				uint32_t q = cells[best];
				cells[pc] = q;
				cells[best] = p;
				faces[p] = best;
				if (q != PME_TOPOLOGY_EMPTY)
					faces[q] = pc;
				moved++;
			}
		}
		if (!moved)
			break;
		for (size_t i = n; i > 1; i--)
		{
			size_t j = random32(&seed) % i;
			uint32_t t = w->order[i - 1];
			w->order[i - 1] = w->order[j];
			w->order[j] = t;
		}
	}
	measure(adj, n, faces, d->width, &r);
	r.mappingHash = UINT64_C(14695981039346656037);
	for (size_t i = 0; i < count; i++)
		for (int b = 0; b < 4; b++)
			r.mappingHash =
				(r.mappingHash ^ ((cells[i] >> (b * 8)) & 255)) * UINT64_C(1099511628211);
	*out = r;
	return PME_OK;
}
