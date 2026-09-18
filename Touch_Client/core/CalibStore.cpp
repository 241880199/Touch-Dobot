#include "CalibStore.h"
#include "../config/Config.h"
#include <cstdio>
#include <cstring>
#include <ctime>
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

    bool isFresh(long savedAtUnix, long nowUnix, long maxAgeSec) {
        if (savedAtUnix <= 0) return false;      // 缺字段 / 解析失败 -> 过期
        long age = nowUnix - savedAtUnix;
        if (age < 0) age = 0;                    // 时钟回拨 -> 当作刚保存
        return age <= maxAgeSec;
    }

    // 极简 JSON 数值字段提取。只认本程序写出的格式 (顶层 "key": number),
    // 不引入第三方 JSON 库。
    static bool readLongField(const char* path, const char* key, long& out) {
        FILE* f = fopen(path, "r");
        if (!f) return false;
        char buf[4096];
        size_t n = fread(buf, 1, sizeof(buf) - 1, f);
        fclose(f);
        if (n == 0) return false;
        buf[n] = '\0';

        char pat[64];
        snprintf(pat, sizeof(pat), "\"%s\"", key);
        const char* p = strstr(buf, pat);
        if (!p) return false;
        p = strchr(p, ':');
        if (!p) return false;
        p++;
        while (*p == ' ' || *p == '\t') p++;
        char* end = nullptr;
        double v = strtod(p, &end);
        if (end == p) return false;
        out = (long)v;
        return true;
    }

    const char* resolve(const char* name) {
        const char* path = fileFor(name);

        FILE* probe = fopen(path, "r");
        if (!probe) return nullptr;     // 不存在: 静默
        fclose(probe);

        long savedAt = 0;
        const bool hasStamp = readLongField(path, "saved_at_unix", savedAt);
        const long now = (long)time(NULL);

        if (hasStamp && isFresh(savedAt, now, Config::CALIB_MAX_AGE_SEC)) {
            return path;
        }

        // 过期 —— 或没有时间戳 (明确决策: 一律作废)
        char newPath[MAX_PATH];
        snprintf(newPath, sizeof(newPath), "%s.expired", path);
        remove(newPath);                // 同名旧文件先清掉, 失败也无所谓
        const bool renamed = (rename(path, newPath) == 0);

        std::printf("\n");
        std::printf("[Calib] !! %s 已作废 — %s\n", name,
                    hasStamp ? "超过有效期" : "缺少 saved_at_unix 时间戳");
        if (hasStamp) {
            std::printf("[Calib] !!   保存于 %.1f 小时前 (上限 %.0f 小时)\n",
                        (double)(now - savedAt) / 3600.0,
                        (double)Config::CALIB_MAX_AGE_SEC / 3600.0);
        }
        if (renamed) std::printf("[Calib] !!   已改名为 %s.expired\n", name);
        else         std::printf("[Calib] !!   (改名失败, 文件保留原处)\n");
        return nullptr;
    }
}
