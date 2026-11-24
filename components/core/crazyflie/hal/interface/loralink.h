#pragma once
#include <stdbool.h>
void loralinkSetGpsReturn(bool enabled);
bool loralinkGpsReturnEnabled(void);
#include "crtp.h"
void loralinkInit(void);
bool loralinkTest(void);
struct crtpLinkOperations *loralinkGetLink(void);
