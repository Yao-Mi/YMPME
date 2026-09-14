#ifndef YMPME_LOG_H
#define YMPME_LOG_H
#include "../CONFIG/YMPME_PubType.h"
#include "../CONFIG/YMPME_DebugConfig.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef enum pme_log_level_ {
    PME_LOG_DEBUG = 0, PME_LOG_INFO, PME_LOG_WARNING, PME_LOG_ERROR, PME_LOG_OFF
} pme_log_level;
/* Borrowed record and strings: valid only during the synchronous callback. */
typedef struct pme_log_record_ {
    pme_log_level level;
    pme_result result;
    const char *message, *file, *function;
    unsigned line;
} pme_log_record;
typedef void (*pme_log_callback)(const pme_log_record* record, void* user);

const char* YMPME_Debug_ResultString(pme_result result);
const char* YMPME_Debug_LevelString(pme_log_level level);
/* One logger per linked PME library. Configure only while all logging is stopped.
 * NULL callback or PME_LOG_OFF mutes output. user must outlive all callbacks.
 * Concurrent calls require a thread-safe callback; callbacks must not reenter logging.
 * Returns INVALID_ARGUMENT for invalid level without changing the configuration. */
pme_result YMPME_Debug_SetLogger(pme_log_callback callback, void* user, pme_log_level minimum);
/* Restore WARNING threshold and stderr output (or silent if compiled without stderr). */
void YMPME_Debug_ResetLogger(void);
void YMPME_Debug_Log(pme_log_level level, pme_result result, const char* message,
                     const char* file, unsigned line, const char* function);
/* Logs non-OK results as ERROR, returning the original result unchanged. */
pme_result YMPME_Debug_Report(pme_result result, const char* message,
                             const char* file, unsigned line, const char* function);
/* No-op unless the library was built with PME_ENABLE_ASSERTS=1; then logs and aborts. */
void YMPME_Debug_AssertFail(const char* expression, const char* file,
                          unsigned line, const char* function);
#ifdef __cplusplus
}
#endif

#if PME_ENABLE_LOGGING
#define YMPME_LOG(level, result, message) \
    YMPME_Debug_Log((level), (result), (message), __FILE__, __LINE__, __func__)
#define YMPME_REPORT(result, message) \
    YMPME_Debug_Report((result), (message), __FILE__, __LINE__, __func__)
#else
/* Disabled logs never evaluate arguments. REPORT still evaluates result exactly once. */
#define YMPME_LOG(level, result, message) ((void)0)
#define YMPME_REPORT(result, message) (result)
#endif
#if PME_ENABLE_ASSERTS
#define YMPME_ASSERT(condition) ((condition) ? (void)0 : \
    YMPME_Debug_AssertFail(#condition, __FILE__, __LINE__, __func__))
#else
#define YMPME_ASSERT(condition) ((void)0)
#endif
#endif
