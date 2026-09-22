#pragma once

// MATLAB 可调的力反射增益 (2026-09-22)。
//
// 【本模块只管目标值】校验 + 持久化 + 防抖落盘。
//   增益【斜坡】(0.25s 平滑到位) 是信号处理, 在 ForcePipeline::step 里, 不在这里 ——
//   所以本模块的 gain() 返回的是【目标值】, MATLAB 回读报的也是它 (否则界面上的数字会自己动)。

namespace ForceTuning {
    // ===== 可取范围 —— 【唯一一份定义】=====
    // MATLAB 滑条的上下限、被拒提示里的数字, 全部由 RG| 回读下发, 不在这里之外再写一遍。
    //   下限 100 ⇒ 界面上试不了 120 以下 (要更低只能改 Config.h)。
    constexpr double GAIN_MIN = 100.0;
    constexpr double GAIN_MAX = 300.0;

    // 落盘前的静默期 (ms)。拖动滑条会产生几十条命令/秒, 不可能每条都写盘。
    // 代价如实说: 拖完立刻杀进程, 最后 1 秒的改动会丢。
    constexpr unsigned long TUNING_DEBOUNCE_MS = 1000;

    // 出厂默认 = Config::FORCE_REFLECTION_GAIN (Config.h:223)。
    // ⚠ 【优先级】calib/force_tuning.json > Config.h 的默认值。
    //   改了 Config.h 却"没反应", 先看这一行和启动横幅打出的来源。
    double defaultGain();

    // 当前生效的【目标值】。线程安全 (atomic)。
    double gain();

    // 校验 [GAIN_MIN, GAIN_MAX] 且有限 → 过则 store + 标脏, 返回 true; 否则不改任何状态。
    bool setGain(double v);

    // 纯函数: 从 json 文本解出 gain。version 必须是 1; 值必须有限且落在范围内。
    // 任何不合规都返回 false (调用方负责出声)。
    bool parseGainJson(const char* text, double* outGain);

    // 显式路径版本 —— 测试用, loadOnStartup / tick 也复用它们。
    bool saveToFile(const char* path, double gain);
    bool loadFromFile(const char* path, double* outGain);

    // 启动时调一次: 用 CalibStore 的路径读, 读到了就装上, 并打一行横幅写明【值 + 来源】。
    // ⚠ 必须在 RelayCore::initRelayReporting() 【之前】调用 —— 理由见 main.cpp 那一处的注释。
    void loadOnStartup();

    // 防抖落盘: 值变过 且 距上次改动 ≥ TUNING_DEBOUNCE_MS 才写。
    // 由 RelayCore::pollRelayCommands() 每帧调用 (借现成的空闲循环当心跳, 不新起线程)。
    void tick();
}
