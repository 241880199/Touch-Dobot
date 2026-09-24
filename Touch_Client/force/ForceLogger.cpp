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
        fprintf(f, "t_ms,Fx,Fy,Fz,Mx,My,Mz,pose_x,pose_y,pose_z,pose_rx,pose_ry,pose_rz,ff_enabled,"
                   "acc_x,acc_y,acc_z,is_still,raw_x,raw_y,raw_z\n");
        fflush(f);
        return true;
    }

    void close() {
        if (g_file) { fclose(g_file); g_file = nullptr; }
    }

    int formatLine(char* buf, size_t len,
                   unsigned long tMs, const double force[6],
                   const double pose[6], int ffEnabled,
                   const double acc[3], int isStill,
                   const double raw[3]) {
        // 精度取 %.4f: Fi 的误差尺度是 ~0.5 N ⇒ acc ~1.2 m/s², 四位小数足够分辨。
        // (力/力矩仍是 %.3f —— 那是传感器本身的量化台阶, 与本列无关。)
        return snprintf(buf, len,
            "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,"
            "%.4f,%.4f,%.4f,%d,%.3f,%.3f,%.3f",
            tMs,
            force[0], force[1], force[2], force[3], force[4], force[5],
            pose[0], pose[1], pose[2], pose[3], pose[4], pose[5],
            ffEnabled,
            acc[0], acc[1], acc[2], isStill,
            raw[0], raw[1], raw[2]);
    }

    void log(unsigned long tMs, const double force[6], const double pose[6], int ffEnabled,
             const double acc[3], int isStill, const double raw[3]) {
        if (!g_file) return;
        char buf[384];
        formatLine(buf, sizeof(buf), tMs, force, pose, ffEnabled, acc, isStill, raw);
        fprintf(g_file, "%s\n", buf);
    }
}
