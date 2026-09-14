#include "DEBUG/YMPME_Log.h"
#include <stdio.h>
static int calls;
static void capture(const pme_log_record* record, void* user) { (void)record; (void)user; ++calls; }
int main(void)
{
    int evaluations=0;
    YMPME_Debug_SetLogger(capture,NULL,PME_LOG_DEBUG);
    YMPME_LOG((++evaluations,PME_LOG_ERROR),(++evaluations,PME_INVALID_ARGUMENT),(++evaluations,"hidden"));
    YMPME_ASSERT(++evaluations);
    if(evaluations) return 1;
    pme_result result=YMPME_REPORT((++evaluations,PME_INVALID_ARGUMENT),(++evaluations,"hidden"));
    if(evaluations!=1 || result!=PME_INVALID_ARGUMENT) return 2;
    YMPME_Debug_Log(PME_LOG_ERROR,PME_INVALID_ARGUMENT,"direct",__FILE__,1,__func__);
    YMPME_Debug_AssertFail("direct",__FILE__,1,__func__);
    if(calls) return 3;
    puts("PASS disabled logs: no callbacks, no macro argument evaluation, result preserved");
    return 0;
}
