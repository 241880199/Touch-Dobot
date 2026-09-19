#include "CalibStore.h"
#include <cstdio>
#include <cstring>
#include <windows.h>

namespace CalibStore {

    bool deriveDir(const char* exePath, char* out, size_t outSize) {
        if (!exePath || !out || outSize == 0) return false;
        char buf[MAX_PATH];
        size_t n = strlen(exePath);
        if (n >= sizeof(buf)) return false;
        memcpy(buf, exePath, n + 1);

        // 去掉文件名
        char* p = strrchr(buf, '\\');
        if (!p) return false;              // 一个分隔符都没有: 推不出目录
        *p = '\0';

        // 再上溯两级。上溯停在盘符根 ("C:\a" -> 不再剥成 "C:"):
        // 盘符根没有可以承载 calib\ 的目录, 而且各种布局的标定文件会撞在一起。
        for (int up = 0; up < 2; up++) {
            char* q = strrchr(buf, '\\');
            if (!q || (q == buf + 2 && buf[1] == ':')) break;
            *q = '\0';
        }
        if (snprintf(out, outSize, "%s\\calib\\", buf) >= (int)outSize) return false;
        return true;
    }

    const char* dir() {
        static char s_dir[MAX_PATH] = {0};
        if (s_dir[0] != '\0') return s_dir;

        char exe[MAX_PATH] = {0};
        if (GetModuleFileNameA(NULL, exe, MAX_PATH) == 0 ||
            !deriveDir(exe, s_dir, sizeof(s_dir))) {
            // 理论不可达; 退回当前目录, 至少不崩
            snprintf(s_dir, sizeof(s_dir), ".\\calib\\");
        }
        CreateDirectoryA(s_dir, NULL);   // 已存在时返回 ERROR_ALREADY_EXISTS, 无害
        return s_dir;
    }

    const char* fileFor(const char* name) {
        static char s_buf[MAX_PATH];
        snprintf(s_buf, sizeof(s_buf), "%s%s", dir(), name);
        return s_buf;
    }
}
