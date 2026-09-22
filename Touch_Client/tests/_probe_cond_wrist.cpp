// 一次性探针（2026-09-22，现场报"某点附近机械臂抖动"时用）
//
// 问：那个点是不是【腕部奇异阻尼】在起作用？
// 做法：拿现场给的关节角，调【生产代码】SingularityAvoidance::dampOrientationMotion，
//       把 delta 方向扫一遍单位球，取 min|damped|/|delta| ——
//       阻尼只缩放"沿 u_min 的分量"，所以那个最小值【就是 β】。
//       β = 1 − t², t = (cond − EARLY)/(BLOCK − EARLY) ⇒ 反解出 cond_wrist。
//
// ⚠ 为什么不去自己复算 cond_wrist：svd3x3 在 SingularityAvoidance.cpp 里是 static，
//   复算要照抄一遍 J_w 抽取 + SVD，抄错了我也不知道。扫球反推 β 用的是【同一份代码】，
//   没有第二份实现可以漂。
#define TEST_SINGAVOID
#include "../safety/SingularityAvoidance.h"
#include "../robot/Kinematics.h"
#include "../config/Config.h"
#include <cstdio>
#include <cmath>

int main() {
    // 现场给的（机器人位姿 R=(+176.1, −4.2, +17.5)deg 时 'd' 锁定打出的）
    double j[6] = { -83.9, 30.5, -85.2, -32.2, 95.3, -191.5 };
    Vec3 target(176.1, -4.2, 17.5);

    Vec3 tcp = Kinematics::forwardPosition(j);
    printf("FK 末端 = (%.2f, %.2f, %.2f) mm\n", tcp.x, tcp.y, tcp.z);

    // 顺带报一下 J5：腕部奇异的经典判据是 J5 -> 0（J4/J6 轴共线）
    printf("J5 = %.1f deg   (|sin J5| = %.4f)\n", j[4], fabs(sin(j[4] * 3.14159265358979 / 180.0)));

    const int N = 36;
    double best = 1e9;
    Vec3 bestDir(0, 0, 0);
    double maxAdj = 0.0, maxRep = 0.0;   // Layer 2 (肩部推开) / Layer 3 有没有动
    for (int i = 0; i <= N; i++) {
        double th = 3.14159265358979 * i / N;
        for (int k = 0; k < 2 * N; k++) {
            double ph = 3.14159265358979 * k / N;
            Vec3 d(sin(th) * cos(ph), sin(th) * sin(ph), cos(th));
            Vec3 tcpAdj, rep;
            Vec3 out = SingularityAvoidance::dampOrientationMotion(target, d, tcp, j, tcpAdj, rep);
            double mag = sqrt(out.x * out.x + out.y * out.y + out.z * out.z);
            if (mag < best) { best = mag; bestDir = d; }
            double a = sqrt(tcpAdj.x * tcpAdj.x + tcpAdj.y * tcpAdj.y + tcpAdj.z * tcpAdj.z);
            double r = sqrt(rep.x * rep.x + rep.y * rep.y + rep.z * rep.z);
            if (a > maxAdj) maxAdj = a;
            if (r > maxRep) maxRep = r;
        }
    }
    // 肩部那一路（Layer 2）看的是肘部到 Z 轴的距离
    Vec3 pos7[7];
    Kinematics::computeJointPositions(j, pos7);
    printf("\n七个关节位置 (x, y, r=hypot(x,y)):\n");
    for (int i = 0; i < 7; i++) {
        double r_ = sqrt(pos7[i].x * pos7[i].x + pos7[i].y * pos7[i].y);
        printf("  pos[%d] = (%9.3f, %9.3f, %9.3f)   r = %10.6f mm\n",
               i, pos7[i].x, pos7[i].y, pos7[i].z, r_);
    }
    double r_elbow = sqrt(pos7[2].x * pos7[2].x + pos7[2].y * pos7[2].y);
    printf("\n★ r_elbow = %.9f mm\n", r_elbow);
    printf("   SAFE_R = %.1f mm (低于此触发)   DUAL_SING = %.1f   CRITICAL_R = %.1f\n",
           Config::SINGAVOID_SHOULDER_SAFE_R, Config::SINGAVOID_DUAL_SING_ELBOW_THR,
           Config::SINGAVOID_SHOULDER_CRITICAL_R);
    printf("   ⇒ 落在【%s】\n",
           r_elbow >= Config::SINGAVOID_DUAL_SING_ELBOW_THR ? "黄区 [80,120)" :
           (r_elbow >= Config::SINGAVOID_SHOULDER_CRITICAL_R ? "橙区 [50,80)" : "红区 <50 —— 最大力度"));
    printf("   推的方向取 (x/r, y/r); r>0.01 才用, 否则硬编码 (1,0)\n");
    printf("   ⇒ 当前 r %s 0.01 ⇒ 方向 = %s\n",
           (r_elbow > 0.01) ? ">" : "<=",
           (r_elbow > 0.01) ? "(x/r, y/r) 【由这个极小向量决定 —— 数值上会甩】" : "(1,0) 硬编码");
    printf("\n整个扫描里 max|tcpAdj| = %.4f mm   max|repulsion| = %.4f N\n", maxAdj, maxRep);

    // ---- 快速 beta: 用【迹】—— damped = (I − (1−β) u uᵀ) delta ⇒ trace = 2 + β ----
    // ⇒ 只需 3 次调用(三个基方向)就能拿到 beta, 于是可以扫关节邻域。
    // 注意: 必须用同一 target/tcp/joints, 且 delta 取单位基向量。
    {
        double s = 0.0;
        for (int a = 0; a < 3; a++) {
            Vec3 e(a == 0 ? 1.0 : 0.0, a == 1 ? 1.0 : 0.0, a == 2 ? 1.0 : 0.0);
            Vec3 tcpAdj, rep;
            Vec3 out = SingularityAvoidance::dampOrientationMotion(target, e, tcp, j, tcpAdj, rep);
            s += out.x * e.x + out.y * e.y + out.z * e.z;
        }
        printf("\n[迹法交叉核对] beta = (sum of damped·e_i) - 2 = %.6f  (扫球法 %.6f)\n", s - 2.0, best);
        if (fabs((s - 2.0) - best) > 1e-6) {
            printf("  ⚠ 两法不一致 ⇒ damped 不止是 I−(1−β)uuᵀ 那一种缩放, 别用迹法扫。\n");
        } else {
            printf("  ✓ 两法一致 ⇒ 可以用迹法快速扫邻域。\n");
        }
    }

    // ---- 扫 J4/J5/J6 邻域: 找 beta<1 (即有阻尼) 有多远 ----
    double worst = 1.0, worstJ[3] = {0, 0, 0};
    int found = 0;
    double minB = 1.0;
    for (double d4 = -60; d4 <= 60; d4 += 5) {
        for (double d5 = -60; d5 <= 60; d5 += 5) {
            for (double d6 = -60; d6 <= 60; d6 += 5) {
                double jj[6] = { j[0], j[1], j[2], j[3] + d4, j[4] + d5, j[5] + d6 };
                double s = 0.0;
                Vec3 tc = Kinematics::forwardPosition(jj);
                for (int a = 0; a < 3; a++) {
                    Vec3 e(a == 0 ? 1.0 : 0.0, a == 1 ? 1.0 : 0.0, a == 2 ? 1.0 : 0.0);
                    Vec3 tcpAdj, rep;
                    Vec3 out = SingularityAvoidance::dampOrientationMotion(target, e, tc, jj, tcpAdj, rep);
                    s += out.x * e.x + out.y * e.y + out.z * e.z;
                }
                double b = s - 2.0;
                if (b < minB - 1e-9) {
                    minB = b;
                    worstJ[0] = d4; worstJ[1] = d5; worstJ[2] = d6;
                }
                if (b < 0.999) found++;
            }
        }
    }
    printf("\n[邻域扫描 J4/J5/J6 各 ±60°, 步长 5°]  共 %d 个位形\n", 25 * 25 * 25);
    printf("  其中 beta < 1 (【有阻尼】) 的位形: %d 个\n", found);
    printf("  最小的 beta = %.6f, 出现在 (J4%+.0f, J5%+.0f, J6%+.0f) 相对当前\n",
           minB, worstJ[0], worstJ[1], worstJ[2]);
    if (found > 0 && minB < 1.0) {
        double tt = sqrt(1.0 - minB);
        printf("  ⇒ 该位形 cond_wrist ≈ %.1f (阈值 20~150)\n", 20.0 + tt * 130.0);
    }

    printf("\nmin|damped| / |delta| = beta = %.6f\n", best);
    printf("  (扫了 %d 个方向; 取到最小的那个 delta 方向 = (%.3f, %.3f, %.3f))\n",
           (N + 1) * 2 * N, bestDir.x, bestDir.y, bestDir.z);

    const double EARLY = Config::SINGAVOID_COND_WRIST_EARLY;
    const double BLOCK = Config::SINGAVOID_COND_WRIST_BLOCK;
    if (best > 0.999) {
        printf("\n⇒ 【没有阻尼】beta = 1 ⇒ cond_wrist < %.1f (早期阈值)。\n", EARLY);
        printf("   ⇒ 该点的抖动【不是】腕部奇异阻尼造成的。\n");
    } else {
        double t = sqrt(1.0 - best);
        double cond = EARLY + t * (BLOCK - EARLY);
        printf("\n⇒ 【阻尼在起作用】beta = %.4f ⇒ 反解 cond_wrist ≈ %.1f\n", best, cond);
        printf("   (阈值: >= %.1f 开始削, >= %.1f 削到 0)\n", EARLY, BLOCK);
        printf("   ⇒ 该点的抖动【可能是】腕部奇异阻尼 —— 但它是否【抖】还取决于\n");
        printf("     cond_wrist 是否在阈值附近来回穿 (位置相关), 以及 u_min 方向是否在转。\n");
    }
    return 0;
}
