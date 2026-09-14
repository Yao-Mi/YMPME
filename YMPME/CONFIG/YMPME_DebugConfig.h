#ifndef YMPME_DEBUG_CONFIG_H
#define YMPME_DEBUG_CONFIG_H
/* Set consistently when building the library and consumers. A prebuilt library
 * cannot be stripped by changing consumer-only macros; rebuild for that. */
#ifndef PME_ENABLE_LOGGING
#define PME_ENABLE_LOGGING 1
#endif
#ifndef PME_DEBUG_STDERR
#define PME_DEBUG_STDERR 1
#endif
#ifndef PME_ENABLE_ASSERTS
#define PME_ENABLE_ASSERTS 0
#endif
#if (PME_ENABLE_LOGGING != 0 && PME_ENABLE_LOGGING != 1) || \
    (PME_DEBUG_STDERR != 0 && PME_DEBUG_STDERR != 1) || \
    (PME_ENABLE_ASSERTS != 0 && PME_ENABLE_ASSERTS != 1)
#error "PME debug options must be 0 or 1"
#endif
#endif
