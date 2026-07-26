#include "../relay/CoordinateTransform.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

// ===== Global calibration state =====
namespace Calibration {
    bool enabled = false;
    double R[9] = {1,0,0, 0,1,0, 0,0,1};
    double t[3] = {0,0,0};
    double rmsError = 0.0;

    bool collectMode = false;
    int  collectCount = 0;
    double collectTouch[MAX_COLLECT_POINTS][3] = {{0}};
    double collectRobot[MAX_COLLECT_POINTS][3] = {{0}};

    bool load(const char* filepath) {
        FILE* f = fopen(filepath, "r");
        if (!f) return false;

        char buf[4096];
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        fclose(f);
        if (n == 0) return false;
        buf[n] = '\0';

        // Parse R array: "R":[v0,v1,...,v8]
        const char* p = strstr(buf, "\"R\":[");
        if (!p) return false;
        p += 4;
        for (int i = 0; i < 9; i++) {
            char* end = nullptr;
            R[i] = strtod(p, &end);
            if (end == p) return false;
            p = end;
            while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
        }

        // Parse t array: "t":[t0,t1,t2]
        p = strstr(buf, "\"t\":[");
        if (!p) return false;
        p += 4;
        for (int i = 0; i < 3; i++) {
            char* end = nullptr;
            t[i] = strtod(p, &end);
            if (end == p) return false;
            p = end;
            while (*p == ',' || *p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
        }

        // Parse rmsError
        p = strstr(buf, "\"rmsError\":");
        if (p) {
            rmsError = strtod(p + 11, nullptr);
        }

        enabled = true;
        return true;
    }

    bool save(const char* filepath) {
        FILE* f = fopen(filepath, "w");
        if (!f) return false;

        fprintf(f, "{\n");
        fprintf(f, "  \"R\": [%.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g, %.15g],\n",
                R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]);
        fprintf(f, "  \"t\": [%.6g, %.6g, %.6g],\n", t[0], t[1], t[2]);
        fprintf(f, "  \"rmsError\": %.6g\n", rmsError);
        fprintf(f, "}\n");
        fclose(f);
        return true;
    }

    void startCollect() {
        collectMode = true;
        collectCount = 0;
    }

    void cancelCollect() {
        collectMode = false;
        collectCount = 0;
    }

} // namespace Calibration
