#pragma once
#include <cstddef>

// 力数据 CSV 落盘 (演示对照实验用)
// 始终记录, 每行含 ff_enabled 标志列, 事后按该标志分组分析。
namespace ForceLogger {
    // 打开 CSV 文件并写入表头。返回 false 表示失败。
    bool open(const char* path);

    // 关闭文件 (幂等)。
    void close();

    // 将一条采样格式化为 CSV 行 (纯函数, 便于单元测试)。
    // 返回写入字符数 (不含结尾 '\0'); 若 >= len 表示被截断。
    int formatLine(char* buf, size_t len,
                   unsigned long tMs,          // 相对启动时间戳 (ms)
                   const double force[6],      // 补偿+滤波后的 6 轴力/力矩 (N, Nm)
                   const double pose[6],       // 机器人末端位姿 x,y,z,rx,ry,rz
                   int ffEnabled);             // 1=力反馈开, 0=关

    // 写一条采样 (文件未打开时忽略)。
    void log(unsigned long tMs, const double force[6], const double pose[6], int ffEnabled);
}
