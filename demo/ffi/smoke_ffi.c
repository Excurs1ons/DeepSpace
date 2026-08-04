/* 编译: cc -I include smoke_ffi.c -L target/debug -ldeepspace -o smoke_ffi
 * 运行: LD_LIBRARY_PATH=target/debug ./smoke_ffi */
#include "deepspace.h"
#include <stdio.h>

int main(void) {
    DSWorld *w = ds_world_create();
    if (!w) { printf("create failed\n"); return 1; }

    DSBallisticConfig tgt = {0};
    snprintf(tgt.name, sizeof tgt.name, "TGT-C");
    tgt.position = (DSVec3){0.0, 6.5e6, 0.0};
    tgt.velocity = (DSVec3){1500.0, 500.0, 0.0};
    tgt.mass = 1000.0; tgt.ref_area_m2 = 0.5; tgt.cd = 0.2;
    long tid = ds_world_add_ballistic(w, &tgt);
    printf("target id=%ld\n", tid);

    for (int i = 0; i < 50; i++) ds_world_step(w);

    size_t n = ds_world_entity_count(w);
    printf("entities=%zu time=%.1f\n", n, ds_world_time(w));
    for (size_t i = 0; i < n; i++) {
        DSEntityState st;
        if (ds_world_entity_at(w, i, &st) == DS_OK)
            printf("  %llu kind=%d alt=%.1f km\n",
                   (unsigned long long)st.id, (int)st.kind, st.altitude_m/1000.0);
    }
    DSEvent evts[8];
    size_t ne = ds_world_poll_events(w, evts, 8);
    for (size_t i = 0; i < ne; i++)
        printf("  [%u] %s\n", evts[i].kind, evts[i].text);

    ds_world_destroy(w);
    printf("C ABI smoke OK\n");
    return 0;
}