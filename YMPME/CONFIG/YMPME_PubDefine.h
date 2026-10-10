#ifndef YMPME_PUBDEFINE_H
#define YMPME_PUBDEFINE_H
#define YMPME_VERSION_MAJOR 0
#define YMPME_VERSION_MINOR 4
#define YMPME_VERSION_PATCH 0
#define YMPME_VERSION_STRINGIFY_(value) #value
#define YMPME_VERSION_STRINGIFY(value) YMPME_VERSION_STRINGIFY_(value)
#define YMPME_VERSION_STRING YMPME_VERSION_STRINGIFY(YMPME_VERSION_MAJOR) "." \
    YMPME_VERSION_STRINGIFY(YMPME_VERSION_MINOR) "." YMPME_VERSION_STRINGIFY(YMPME_VERSION_PATCH)
#ifdef __cplusplus
extern "C" {
#endif
/* Version baked into the linked library; compare against YMPME_VERSION_STRING. */
const char *YMPME_VersionString(void);
#ifdef __cplusplus
}
#endif
#define YMPME_DEFAULT_STEP (1.0f / 120.0f)
#define YMPME_DEFAULT_ITERATIONS 8u
#endif
