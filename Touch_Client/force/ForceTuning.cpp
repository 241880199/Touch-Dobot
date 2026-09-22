#include "ForceTuning.h"
#include "../config/Config.h"
#include "../core/CalibStore.h"
#include "../core/JsonLite.h"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <windows.h>

namespace ForceTuning {

// ★★ 静态初值直接来自 Config.h, 【不读文件】。
//   这是"测试不被现场调参污染"的落点: 只有显式的 loadOnStartup() 才会改它,
//   而测试【不调】loadOnStartup() (见 tests/test_force_tuning.cpp 顶部那段)。
static std::atomic<double> s_gain{ Config::FORCE_REFLECTION_GAIN };

// 防抖状态。只由 GLUT idle 线程碰 (setGain 与 tick 都在那里) ⇒ 不需要原子。
// 用独立的 bool 而不是"时间戳 == 0"当哨兵: 开机后第一秒 GetTickCount() 可能真是 0。
static bool  s_dirty = false;
static DWORD s_dirtyMs = 0;

double defaultGain() { return Config::FORCE_REFLECTION_GAIN; }
double gain() { return s_gain.load(); }

static bool inRange(double v) {
    return std::isfinite(v) && v >= GAIN_MIN && v <= GAIN_MAX;
}

bool setGain(double v) {
    if (!inRange(v)) return false;
    s_gain.store(v);
    s_dirty = true;
    s_dirtyMs = GetTickCount();
    return true;
}

bool parseGainJson(const char* text, double* outGain) {
    if (!text || !outGain) return false;

    // 【版本先判】顺序要紧: 先读值再判版本, 就会在返回 false 之前把半个结果写进调用方。
    // 教训出处: ForceCalibration::loadFromFile 顶上那段 (同一个理由)。
    const char* pv = JsonLite::find(text, "\"version\"");
    if (!pv) return false;
    if (strtol(pv, nullptr, 10) != 1) return false;

    const char* pg = JsonLite::find(text, "\"reflection_gain\"");
    if (!pg) return false;
    char* end = nullptr;
    const double v = strtod(pg, &end);
    if (end == pg) return false;   // 那里根本不是个数
    // ⚠ strtod 会把 "nan" / "inf" 都解析成功, 所以挡它们这件事不能指望 strtod 自己。
    // ★ 2026-09-22 【负对照实测, 结论与直觉相反】: 把 inRange 里的 isfinite 删掉,
    //   test_force_tuning 照样 11/0 全绿 —— 因为 NaN 的任何比较都为假、±inf 必有一侧
    //   越界 ⇒ 光靠后面那对区间比较就已经把非有限数全挡住了。
    //   ⇒ isfinite 是【纵深防御, 不是承重墙】, 删掉它没有任何用例会红。
    //     留着它的理由不是"现在需要", 而是"日后若有人把区间判断改写成别的形状
    //     (例如 `!(v < GAIN_MIN || v > GAIN_MAX)` —— 那一版 NaN 两侧比较都为假,
    //      整个式子为真 ⇒ NaN 会被放行), 它才是最后一道"。
    //     ⚠ 例子的形状要紧: `fabs(v) <= GAIN_MAX` 那种写法【照样】挡住 NaN,
    //       拿它当例子反而不支持本段结论 (评审 2026-09-22 指出, 已改成测试里那一版)。
    //     别把它当成被测试覆盖了的代码。
    if (!inRange(v)) return false;

    *outGain = v;
    return true;
}

bool saveToFile(const char* path, double g) {
    FILE* f = fopen(path, "w");
    if (!f) return false;
    fprintf(f, "{\n  \"version\": 1,\n  \"reflection_gain\": %.6g\n}\n", g);
    fclose(f);
    return true;
}

bool loadFromFile(const char* path, double* outGain) {
    FILE* f = fopen(path, "r");
    if (!f) return false;   // 文件不存在 = 还没调过, 正常路径, 不吵
    char buf[1024];
    const size_t n = fread(buf, 1, sizeof(buf) - 1, f);
    fclose(f);
    if (n == 0) {
        fprintf(stderr, "[Tuning] !! force_tuning.json 【是空文件, 已忽略】: %s\n", path);
        fflush(stderr);
        return false;
    }
    buf[n] = '\0';
    if (!parseGainJson(buf, outGain)) {
        // 响亮地说出"这份文件不是本格式 / 值不可用" —— 不许安静地退化成"用默认值"。
        // 为什么必须响: "用默认值"与"读到了默认值"在控制台上长得一样, 而它们要做的事
        //   完全不同 (前者要去查文件, 后者什么都不用做)。
        fprintf(stderr,
                "[Tuning] !! force_tuning.json 【不可用, 已忽略】: %s\n"
                "[Tuning] !!   期望 { \"version\": 1, \"reflection_gain\": <%.0f..%.0f 之间的有限数> }\n"
                "[Tuning] !!   处置: 本次用 Config.h 的默认值。要重建这份文件, 在 MATLAB 上拖一下滑条即可。\n",
                path, GAIN_MIN, GAIN_MAX);
        fflush(stderr);
        return false;
    }
    return true;
}

void loadOnStartup() {
    const char* path = CalibStore::fileFor("force_tuning.json");
    double v = 0.0;
    if (loadFromFile(path, &v)) {
        s_gain.store(v);
        printf("[Tuning] 力反射增益 = %.1f (来源: %s)\n", v, path);
    } else {
        // 走到这里有两种可能: 文件不存在 (不吵过, 正常), 或文件在但不可用 (上面已响亮报过)。
        // 这一行把【来源】写出来, 这样"改了 Config.h 却没反应"在第一行日志里就有答案。
        printf("[Tuning] 力反射增益 = %.1f (来源: Config.h 默认值; 未采用 %s)\n",
               defaultGain(), path);
    }
}

void tick() {
    if (!s_dirty) return;
    const DWORD now = GetTickCount();
    if ((now - s_dirtyMs) < TUNING_DEBOUNCE_MS) return;   // 还在动, 再等等

    const double v = s_gain.load();
    const char* path = CalibStore::fileFor("force_tuning.json");
    if (saveToFile(path, v)) {
        printf("[Tuning] 力反射增益已落盘: %.1f\n", v);
    } else {
        // 落盘失败必须出声 —— 否则"我调好了"与"下次开机没了"之间没有任何提示。
        fprintf(stderr, "[Tuning] !! 力反射增益【落盘失败】: %s —— 本次会话有效, 重启会丢\n", path);
        fflush(stderr);
    }
    s_dirty = false;
}

} // namespace ForceTuning
