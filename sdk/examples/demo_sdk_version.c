#include "YMPME.h"
#include <stdio.h>
#include <string.h>
#ifndef PME_EXPECTED_VERSION
#error "Build must supply its configured version"
#endif
int main(void) {
  if (strcmp(YMPME_VERSION_STRING, PME_EXPECTED_VERSION) ||
      strcmp(YMPME_VersionString(), PME_EXPECTED_VERSION)) {
    fprintf(stderr, "Version mismatch: header=%s library=%s build=%s\n",
            YMPME_VERSION_STRING, YMPME_VersionString(), PME_EXPECTED_VERSION);
    return 1;
  }
  printf("YMPME %s\n", YMPME_VersionString());
  return 0;
}
