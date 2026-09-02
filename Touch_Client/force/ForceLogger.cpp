#include "ForceLogger.h"
#include <cstdio>

namespace {
    FILE* g_file = nullptr;
}

namespace ForceLogger {
    bool open(const char* path) {
        close();
        FILE* f = fopen(path, "w");
        if (!f) return false;
        g_file = f;
        fprintf(f, "t_ms,Fx,Fy,Fz,Mx,My,Mz,pose_x,pose_y,pose_z,pose_rx,pose_ry,pose_rz,ff_enabled\n");
        fflush(f);
        return true;
    }

    void close() {
        if (g_file) { fclose(g_file); g_file = nullptr; }
    }

    int formatLine(char* buf, size_t len,
                   unsigned long tMs, const double force[6],
                   const double pose[6], int ffEnabled) {
        return snprintf(buf, len,
            "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d",
            tMs,
            force[0], force[1], force[2], force[3], force[4], force[5],
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5],
            ffEnabled);
    }

    void log(unsigned long tMs, const double force[6], const double pose[6], int ffEnabled) {
        if (!g_file) return;
        char buf[256];
        formatLine(buf, sizeof(buf), tMs, force, pose, ffEnabled);
        fprintf(g_file, "%s\n", buf);
    }
}
