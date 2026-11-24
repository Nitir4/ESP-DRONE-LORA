#include "gps_monitor.h"
#include <string.h>
void gps_monitor_start(void) {}
void gps_monitor_snapshot(gps_snapshot_t *out)
{
    memset(out,0,sizeof(*out));
    out->age_ms=UINT32_MAX;
}
