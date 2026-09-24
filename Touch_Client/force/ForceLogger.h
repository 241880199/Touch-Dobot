#pragma once
#include <cstddef>

// 力数据 CSV 落盘 (演示对照实验用)
// 始终记录, 每行含 ff_enabled 标志列, 事后按该标志分组分析。
//
// ===== 列布局 (2026-09-21 起 18 列; 此前 14 列) =====
//   t_ms, Fx,Fy,Fz,Mx,My,Mz, pose_x,pose_y,pose_z,pose_rx,pose_ry,pose_rz, ff_enabled,
//   acc_x,acc_y,acc_z, is_still
// ★ 新列一律【追加在末尾】: 位置读者 (按下标取前 14 列) 不受影响。
//   读的人要能处理两种布局 —— 按表头判断, 别按列数硬编码。
// ★ 追加 acc/is_still 的【理由】(run-005 §14 第 6 条 / §7.6(c)): Fi = mass×acc 的运动时
//   误差只能靠【外部反算】验证 —— 本文件的 pose 列数值二阶差分就是"真加速度", 与 acc 列
//   一比, 才知道 dt 那处修得对不对。**两次手拖比 ΔFi 是【不可比】的**(运动不同), 必须
//   在【同一个运动】上做自洽比较。
//   ⚠ 单位: acc 是 m/s² (MotionEstimator 内部已把 pose 从 mm 换成 m)。
//     **而本文件的 pose 列是 mm** ⇒ 离线用 pose 反算时先 ×0.001 再与 acc 比, 否则差 1000 倍。
//   ⚠ is_still = 1 时 【Fi 不施加】(代价: 不补惯量; 好处: 也不注入假力) ⇒ 拿 acc 算 Fi 之前
//     先看这一列。
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
                   const double pose[6],       // 机器人末端位姿 x,y,z,rx,ry,rz (mm, deg)
                   int ffEnabled,              // 1=力反馈开, 0=关
                   const double acc[3],        // 运动估计器滤波后加速度 (m/s²)
                   int isStill, const double raw[3]);   // raw = 原始 @1304 三轴 (N) —— 见 RelayCore 调用点那段理由               // 1=估计器判"静止" (那时 Fi 不施加)

    // 写一条采样 (文件未打开时忽略)。
    void log(unsigned long tMs, const double force[6], const double pose[6], int ffEnabled,
             const double acc[3], int isStill, const double raw[3]);
}
