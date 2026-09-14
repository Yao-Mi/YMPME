#include "YMPME.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"line %d: %s\n",__LINE__,#x); return 1; } } while(0)
static int calls, failAfter = -1, allocations;
static pme_log_record last;
static void capture(const pme_log_record* record, void* user)
{
    ++*(int*)user;
    last = *record; /* Messages used by this test have static storage. */
}
void* PME_malloc0(size_t bytes)
{
    if(failAfter == 0) return NULL;
    if(failAfter > 0) --failAfter;
    void* p = malloc(bytes);
    if(p) ++allocations;
    return p;
}
void PME_free0(void* p) { if(p) --allocations; free(p); }
int main(void)
{
    CHECK(strcmp(YMPME_Debug_ResultString(PME_OUT_OF_MEMORY),"Out of memory")==0);
    CHECK(strcmp(YMPME_Debug_ResultString((pme_result)123),"Unknown result")==0);
    CHECK(strcmp(YMPME_Debug_LevelString(PME_LOG_WARNING),"WARNING")==0);
    CHECK(strcmp(YMPME_Debug_LevelString((pme_log_level)-1),"UNKNOWN")==0);
    CHECK(YMPME_Debug_SetLogger(capture,&calls,PME_LOG_WARNING)==PME_OK);
    CHECK(YMPME_Debug_SetLogger(NULL,NULL,(pme_log_level)99)==PME_INVALID_ARGUMENT);
    YMPME_LOG(PME_LOG_INFO,PME_OK,"filtered"); CHECK(calls==0);
    unsigned line=__LINE__+1;
    YMPME_LOG(PME_LOG_WARNING,PME_OK,"warning");
#if PME_ENABLE_LOGGING
    CHECK(calls==1 && last.line==line && last.level==PME_LOG_WARNING);
    CHECK(strcmp(last.function,"main")==0 && strstr(last.file,"test_debug.c"));
    CHECK(strcmp(last.message,"warning")==0);
#else
    (void)line; CHECK(calls==0);
#endif
    int before=calls;
    CHECK(YMPME_REPORT(PME_OK,"success")==PME_OK && calls==before);
    pme_body body={0}; pme_body_desc desc=YMPME_Body_DefaultDesc(); desc.mass=0;
    CHECK(YMPME_Body_Init(&body,&desc)==PME_INVALID_ARGUMENT && !body.active);
#if PME_ENABLE_LOGGING
    CHECK(calls==before+1 && last.result==PME_INVALID_ARGUMENT);
    CHECK(strcmp(last.function,"YMPME_Body_Init")==0 && strstr(last.message,"mass"));
#endif
    pme_world w; pme_body storage[1];
    CHECK(YMPME_World_Init(&w,storage,1,NULL)==PME_OK);
    desc=YMPME_Body_DefaultDesc();
    CHECK(YMPME_World_AddBody(&w,&desc));
    CHECK(!YMPME_World_AddBody(&w,&desc));
#if PME_ENABLE_LOGGING
    CHECK(last.result==PME_CAPACITY_EXCEEDED && strstr(last.message,"slots"));
#endif
    CHECK(YMPME_World_Step(&w,0)==PME_INVALID_ARGUMENT && w.stepCount==0);
    pme_vec3 vertices[]={{0,0,0},{1,0,0},{0,1,0},{0,0,1}}, faces[4], edges[12];
    PME_Index indices[]={0,2,1,0,1,3,0,3,2,1,2,3}; pme_convex_mesh mesh={0};
    indices[0]=4;
    CHECK(YMPME_ConvexMesh_Init(&mesh,vertices,4,indices,4,faces,4,edges,12)==PME_INVALID_ARGUMENT);
#if PME_ENABLE_LOGGING
    CHECK(strstr(last.message,"index") && mesh.vertexCount==0);
#endif
    indices[0]=0;
    CHECK(YMPME_ConvexMesh_Init(&mesh,vertices,4,indices,4,faces,0,edges,12)==PME_CAPACITY_EXCEEDED);
#if PME_ENABLE_LOGGING
    CHECK(strstr(last.message,"Face-axis"));
#endif
    for(int i=0;i<2;++i) {
        failAfter=i;
        CHECK(!YMPME_Create_World(2,NULL) && allocations==0);
#if PME_ENABLE_LOGGING
        CHECK(last.result==PME_OUT_OF_MEMORY);
#endif
    }
    failAfter=-1;
#if PME_ENABLE_DYNAMICS
    CHECK(YMPME_Dynamics_Step(NULL,.01f,1)==PME_INVALID_ARGUMENT);
#if PME_ENABLE_LOGGING
    CHECK(strcmp(last.function,"YMPME_Dynamics_Step")==0);
#endif
#endif
    CHECK(YMPME_Debug_SetLogger(capture,&calls,PME_LOG_OFF)==PME_OK);
    before=calls; YMPME_LOG(PME_LOG_ERROR,PME_INVALID_ARGUMENT,"muted"); CHECK(calls==before);
    CHECK(YMPME_Debug_SetLogger(NULL,NULL,PME_LOG_DEBUG)==PME_OK);
    YMPME_LOG(PME_LOG_ERROR,PME_INVALID_ARGUMENT,"muted null"); CHECK(calls==before);
    CHECK(YMPME_Debug_SetLogger(capture,&calls,PME_LOG_DEBUG)==PME_OK);
    YMPME_Debug_Log(PME_LOG_ERROR,PME_OK,NULL,NULL,7,NULL);
#if PME_ENABLE_LOGGING
    CHECK(calls==before+1 && !*last.message && !*last.file && !*last.function);
#endif
    before=calls;
    YMPME_Debug_Log((pme_log_level)-1,PME_OK,"bad",__FILE__,1,__func__);
    YMPME_Debug_Log(PME_LOG_OFF,PME_OK,"bad",__FILE__,1,__func__);
    CHECK(calls==before);
    int evaluated=0; pme_result result=YMPME_REPORT((++evaluated,PME_INVALID_ARGUMENT),"once");
    CHECK(evaluated==1 && result==PME_INVALID_ARGUMENT);
    YMPME_Debug_ResetLogger();
    puts("PASS debug: callback context, filtering, location, errors, allocation rollback and muting");
    return 0;
}
