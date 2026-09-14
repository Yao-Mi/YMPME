#include "DEBUG/YMPME_Log.h"
#include <string.h>
#if defined(__unix__)
#include <sys/resource.h>
#endif
int main(int argc,char** argv)
{
#if defined(__unix__)
    struct rlimit limit={0,0}; (void)setrlimit(RLIMIT_CORE,&limit);
#endif
    if(argc!=2) return 2;
    if(strcmp(argv[1],"assert_muted")==0) {
        YMPME_Debug_SetLogger(NULL,NULL,PME_LOG_OFF);
        YMPME_ASSERT(0 && "muted-assert-probe"); return 0;
    }
    if(strcmp(argv[1],"assert")==0) { YMPME_ASSERT(0 && "assert-probe"); return 0; }
    if(strcmp(argv[1],"stderr")==0) {
        YMPME_Debug_SetLogger(NULL,NULL,PME_LOG_OFF);
        YMPME_LOG(PME_LOG_ERROR,PME_INVALID_ARGUMENT,"MUTED");
        YMPME_Debug_ResetLogger();
        YMPME_LOG(PME_LOG_INFO,PME_OK,"FILTERED");
        YMPME_LOG(PME_LOG_ERROR,PME_INVALID_ARGUMENT,"stderr-probe");
        return 0;
    }
    return 3;
}
