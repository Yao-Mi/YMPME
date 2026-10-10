#ifndef GRASP_LAB_H
#define GRASP_LAB_H
#include "DYNAMICS/YMPME_Dynamics.h"
#include "SENSOR/TACTILE/YMPME_SurfaceTactile.h"
#define GRASP_DT (1.f / 240)
#define GRASP_CELLS 128
enum
{
	GRASP_CARRIAGE,
	GRASP_LEFT,
	GRASP_RIGHT,
	GRASP_OBJECT,
	GRASP_GROUND
};
typedef struct grasp_geometry_
{
	pme_convex_mesh mesh;
	pme_vec3 vertices[16], faceAxes[28], edgeAxes[84];
	PME_Index indices[84];
	pme_vec3 inertia;
} grasp_geometry;
int Grasp_Geometry(grasp_geometry *g, int shape, float mass);
typedef struct grasp_lab_
{
	pme_rigid_body bodies[5];
	pme_joint joints[3];
	pme_collider colliders[4];
	pme_dynamic_contact contacts[32];
	pme_dynamics world;
	pme_surface_patch patches[2][GRASP_CELLS];
	pme_vec3 forces[2][GRASP_CELLS];
	pme_surface_taxel cells[2][GRASP_CELLS];
	pme_surface_tactile_frame tactile[2];
	pme_vec3 fingerForce[2], gripForce, groundForce;
	pme_rotation sampleRotation[2];
	float targetAngle, targetHeight, friction, motorLimit;
	int mode, automatic, motors, shape; /* 0 sphere, 1 box, 2 convex mesh */
	grasp_geometry geometry;
	pme_wrench objectContact;
	uint64_t steps;
} grasp_lab;
/* mode 0: grip/lift/release, mode 1: one finger pushes a rolling ball. */
int Grasp_Init(grasp_lab *s, int mode);
int Grasp_InitShape(grasp_lab *s, int mode, int shape);
int Grasp_Step(grasp_lab *s);
/* Only the two fingers carry 8x16 tactile arrays. The manipulated object is
 * an ordinary rigid body with no skin. objectContact is direct solver feedback
 * about its start-of-step COM. Finger cell area is cached once at initialization. */
size_t Grasp_CellCount(int sensor);
float Grasp_NormalForce(const grasp_lab *s, int sensor);
#endif
