#include "YMPME_Log.h"
#if PME_ENABLE_LOGGING && PME_DEBUG_STDERR
#include <stdio.h>
#endif
#if PME_ENABLE_ASSERTS
#include <stdlib.h>
#endif
const char* YMPME_Debug_ResultString(pme_result result)
{
    switch(result) {
    case PME_OK: return "OK";
    case PME_INVALID_ARGUMENT: return "Invalid argument";
    case PME_CAPACITY_EXCEEDED: return "Capacity exceeded";
    case PME_OUT_OF_MEMORY: return "Out of memory";
    case PME_ASSERTION_FAILED: return "Assertion failed";
    default: return "Unknown result";
    }
}
const char* YMPME_Debug_LevelString(pme_log_level level)
{
    switch(level) {
    case PME_LOG_DEBUG: return "DEBUG";
    case PME_LOG_INFO: return "INFO";
    case PME_LOG_WARNING: return "WARNING";
    case PME_LOG_ERROR: return "ERROR";
    case PME_LOG_OFF: return "OFF";
    default: return "UNKNOWN";
    }
}
#if PME_ENABLE_LOGGING
#if PME_DEBUG_STDERR
static void stderrLogger(const pme_log_record* r, void* user)
{
    (void)user;
    fprintf(stderr, "[PME %s] %s (%d): %s [%s:%u %s]\n",
        YMPME_Debug_LevelString(r->level), YMPME_Debug_ResultString(r->result),
        (int)r->result, r->message, r->file, r->line, r->function);
}
#define DEFAULT_LOGGER stderrLogger
#else
#define DEFAULT_LOGGER NULL
#endif
static pme_log_callback logger = DEFAULT_LOGGER;
static void* loggerUser;
static pme_log_level minimumLevel = PME_LOG_WARNING;
#endif
pme_result YMPME_Debug_SetLogger(pme_log_callback callback, void* user, pme_log_level minimum)
{
    if(minimum < PME_LOG_DEBUG || minimum > PME_LOG_OFF) return PME_INVALID_ARGUMENT;
#if PME_ENABLE_LOGGING
    logger = callback;
    loggerUser = user;
    minimumLevel = minimum;
#else
    (void)callback; (void)user;
#endif
    return PME_OK;
}
void YMPME_Debug_ResetLogger(void)
{
#if PME_ENABLE_LOGGING
    logger = DEFAULT_LOGGER;
    loggerUser = NULL;
    minimumLevel = PME_LOG_WARNING;
#endif
}
void YMPME_Debug_Log(pme_log_level level, pme_result result, const char* message,
                     const char* file, unsigned line, const char* function)
{
#if PME_ENABLE_LOGGING
    if(!logger || level < PME_LOG_DEBUG || level >= PME_LOG_OFF || level < minimumLevel) return;
    pme_log_record record = {level, result, message ? message : "", file ? file : "",
                             function ? function : "", line};
    logger(&record, loggerUser);
#else
    (void)level; (void)result; (void)message; (void)file; (void)line; (void)function;
#endif
}
pme_result YMPME_Debug_Report(pme_result result, const char* message,
                             const char* file, unsigned line, const char* function)
{
    if(result != PME_OK) YMPME_Debug_Log(PME_LOG_ERROR, result, message, file, line, function);
    return result;
}
void YMPME_Debug_AssertFail(const char* expression, const char* file,
                          unsigned line, const char* function)
{
#if PME_ENABLE_ASSERTS
    YMPME_Debug_Log(PME_LOG_ERROR, PME_ASSERTION_FAILED, expression, file, line, function);
    abort();
#else
    (void)expression; (void)file; (void)line; (void)function;
#endif
}
