#include "YMPME.h"
#include <stdio.h>
#include <string.h>
static unsigned received;
static void printLog(const pme_log_record* r, void* user)
{
    ++*(unsigned*)user;
    printf("[%s] %s: %s (%s:%u %s)\n", YMPME_Debug_LevelString(r->level),
        YMPME_Debug_ResultString(r->result), r->message, r->file, r->line, r->function);
}
int main(void)
{
    if(YMPME_Debug_SetLogger(printLog,&received,PME_LOG_WARNING)!=PME_OK) return 1;
    YMPME_LOG(PME_LOG_INFO,PME_OK,"This INFO message is filtered");
    pme_body body={0}; pme_body_desc desc=YMPME_Body_DefaultDesc();
    desc.mass=0;
    if(YMPME_Body_Init(&body,&desc)!=PME_INVALID_ARGUMENT || body.active) return 2;
#if PME_ENABLE_LOGGING
    if(received!=1) return 3;
#endif
    desc.mass=1;
    if(YMPME_Body_Init(&body,&desc)!=PME_OK) return 4;
    YMPME_Debug_SetLogger(NULL,NULL,PME_LOG_OFF);
    YMPME_LOG(PME_LOG_ERROR,PME_INVALID_ARGUMENT,"This error is muted");
#if PME_ENABLE_LOGGING
    if(received!=1) return 5;
#endif
    YMPME_Debug_ResetLogger();
    puts("PASS diagnostics: error returned without exiting; corrected input succeeds");
    return 0;
}
