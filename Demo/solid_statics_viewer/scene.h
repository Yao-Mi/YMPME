#ifndef SOLID_DEMO_SCENE_H
#define SOLID_DEMO_SCENE_H
#include "STATICS/YMPME_SolidStatics.h"
#define SOLID_NODES 729
#define SOLID_TETS 3072
#define SOLID_FACES (4 * SOLID_TETS)
#define SOLID_WORK (21 * SOLID_NODES + 13 * SOLID_TETS)
typedef struct {
  size_t node[3], owner, other;
} SolidFace;
typedef struct {
  pme_solid_node nodes[SOLID_NODES];
  pme_solid_tet tets[SOLID_TETS];
  pme_solid_node_result nr[SOLID_NODES];
  pme_solid_tet_result er[SOLID_TETS];
  SolidFace faces[SOLID_FACES];
  double workspace[SOLID_WORK];
  size_t nc, ec, fc;
  int kind, fine, patch, div[3];
  double length[3], load, amp;
  pme_solid_report report;
  pme_solid_status status;
  double max_stress, max_displacement, max_energy;
} SolidScene;
int SolidScene_FaceLoaded(const SolidScene *s, const SolidFace *f);
int SolidScene_Build(SolidScene *s, int kind, int fine, int patch, double load);
int SolidScene_Solve(SolidScene *s);
#endif
