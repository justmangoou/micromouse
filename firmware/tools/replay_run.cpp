// Replays a logged run (tools/read_run.py --csv) through the Navigator's random-block detection, with the course's own
// settings (include/course_params.h), and reports the blocks it would find. A clean run (no random block) must report none.
//
//   build:  g++ -std=gnu++17 -O1 -DNAV_LOW_MEMORY -I lib/course_nav/include -I lib/base/include -I include \
//               tools/replay_run.cpp lib/course_nav/src/planner.cpp lib/course_nav/src/navigator.cpp \
//               lib/course_nav/src/geometry.cpp -o /tmp/replay_run
//   use:    /tmp/replay_run run.csv [baseline.bin]           blocks found (baseline: tools/make_baseline.py --bin)
//           /tmp/replay_run run.csv [baseline.bin] --hits    also print every short reading's hit point
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include "config.h"
#include "course_params.h"
#include "map_data.h"
#include "navigator.h"

namespace {

struct Record {
    int ms, x, y, thMrad, alphaMrad, targetL, targetR, measuredL, measuredR, tofR, tofF, tofL;
};

std::vector<Record> loadRun(const char* path) {
    std::vector<Record> rows;
    FILE* f = fopen(path, "r");
    if (!f) return rows;
    char line[256];
    fgets(line, sizeof line, f);  // header
    Record r;
    while (fgets(line, sizeof line, f))
        if (sscanf(line, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", &r.ms, &r.x, &r.y, &r.thMrad, &r.alphaMrad, &r.targetL,
                   &r.targetR, &r.measuredL, &r.measuredR, &r.tofR, &r.tofF, &r.tofL) == 12)
            rows.push_back(r);
    fclose(f);
    return rows;
}

std::vector<nav::BaselineEntry> loadBaseline(const char* path) {
    std::vector<nav::BaselineEntry> entries;
    FILE* f = fopen(path, "rb");
    if (!f) return entries;
    nav::BaselineEntry e;
    while (fread(&e, sizeof e, 1, f) == 1) entries.push_back(e);
    fclose(f);
    return entries;
}

nav::Navigator gNav(cfg::TOF_LAYOUT, course::navParams());

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s run.csv [baseline.bin] [--hits]\n", argv[0]);
        return 2;
    }
    const std::vector<Record> run = loadRun(argv[1]);
    std::vector<nav::BaselineEntry> baseline;
    bool printHits = false;
    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "--hits") == 0) printHits = true;
        else baseline = loadBaseline(argv[i]);
    }

    gNav.init(MAP_BLOCKS, MAP_BLOCK_COUNT);
    if (!baseline.empty()) {
        nav::Baseline b;
        b.entries = baseline.data();
        b.count = (int)baseline.size();
        gNav.setBaseline(b);
    }
    gNav.start({course::START_X, course::START_Y, course::START_TH}, 0.0f);

    int found = 0;
    for (const Record& r : run) {
        const math::Pose pose = {(float)r.x, (float)r.y, r.thMrad / 1000.0f};
        const float tof[3] = {(float)r.tofR, (float)r.tofF, (float)r.tofL};
        float duty[2];
        gNav.step(pose, (r.measuredL + r.measuredR) / 2.0f, tof, r.ms / 1000.0f, duty);

        if (printHits)
            for (int k = 0; k < 3; k++) {
                if (gNav.shortStreak(k) < 1 || tof[k] >= 800) continue;
                const tof::Mount& m = cfg::TOF_LAYOUT.sensors[k];
                const float c = cosf(pose.th), s = sinf(pose.th), a = pose.th + m.angle;
                const float ox = pose.x + c * m.x - s * m.y, oy = pose.y + s * m.x + c * m.y;
                printf("HIT,%.0f,%.0f,%d\n", ox + tof[k] * cosf(a), oy + tof[k] * sinf(a), k);
            }
        if (gNav.unmappedBlocks() != found) {
            found = gNav.unmappedBlocks();
            const math::Vec2 c = found > 0 ? gNav.planner().extra(found - 1) : math::Vec2{0, 0};
            printf("  t=%.2fs pose(%d,%d,%.2f) tof R%d F%d L%d -> extra #%d at (%.0f,%.0f)\n", r.ms / 1000.0, r.x, r.y,
                   r.thMrad / 1000.0, r.tofR, r.tofF, r.tofL, found, c.x, c.y);
        }
    }
    printf("  total extras: %d, replans %d\n", gNav.unmappedBlocks(), gNav.replans());
    return 0;
}
