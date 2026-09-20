#pragma once

// 上机诊断的【文档化】—— 把一次求解的整屏输出包成一个自描述的 Markdown 块, 追加到
// calib\calib_report.md。
//
// 为什么要有它 (2026-09-19): 上机记录一直是【人肉转录】的 —— Docs\superpowers\specs\
// 2026-09-19-raw-channel-calibration-run-*.md 里"附: 原始控制台输出"那一节整段是手抄的。
// 控制台会滚掉; calib_log.txt 只有 22 个窄列; calib_poses.txt 只有三路原始数据 ——
// 【没有一处留下"操作员当时看到的整屏"】。
//
// 本模块 = 落盘这一半的全部机制, 三件事:
//   · 块格式 (纯函数): 块头 / ```text 围栏 / 逐字正文 / 尾节
//   · 追加写: 【只用 "a+", 永不截断】
//   · stderr 捕获窗口: 库打到 stderr 的那一段 (逐姿态残差表 / [Payload] 自检行) 也并进块里
// 全部是纯的或自包含的, 所以【没有实机也能单测】: tests\test_session_report.cpp。
//
// 【屏幕那一半不在这里, 在 main.cpp 的 BiasCheck::diagEmit】—— 那里保证屏幕写出去的字节与
// 交给本模块的正文是【同一份】, 不是"再打印一遍到文件"。两处各打一遍是会让两者漂移的,
// 而本项目的一条验收判据恰恰是"控制台与落盘必须一致"。

#include <cstdio>
#include <cstring>
#include <string>
#include <cerrno>      // errno / EEXIST (临时文件撞名 -> 换名重试, 见 stderrCaptureBegin)
#ifdef _WIN32
#include <io.h>        // _dup / _dup2 / _close / _open / _read / _unlink
#include <fcntl.h>     // _O_CREAT / _O_EXCL / _O_WRONLY / _O_RDONLY / _O_BINARY
#include <sys/stat.h>  // _S_IREAD / _S_IWRITE
#include <process.h>   // _getpid (临时文件名里的进程号)
#endif

namespace SessionReport {

    // ===== 块格式 (纯函数) =====

    // 块头。ts 的格式与 calib_poses.txt 的 "# attempt" 行【逐字相同】
    // (localtime + "%Y-%m-%d %H:%M:%S"), 两份文件因此能按时间对上:
    //   → "## 采集 2026-09-19 15:33:38 — 10 姿态 / 5 对"
    // 用 string 拼而不是 snprintf: 没有截断的可能 (ts 长短不受控, 截断会静默毁掉时间戳)。
    inline std::string blockHeader(const char* ts, int poses, int pairs) {
        std::string s = "## 采集 ";
        s += (ts && *ts) ? ts : "(无时间戳)";
        s += " — ";
        s += std::to_string(poses);
        s += " 姿态 / ";
        s += std::to_string(pairs);
        s += " 对";
        return s;
    }

    // 一个完整的块 = 块头 + 空行 + ```text 围栏 + 【正文逐字】 + 围栏 + 尾节。
    //   body    = 该次求解控制台输出的逐字全文 (与屏幕【同一份字节】, 见 main.cpp 的 diagEmit)
    //   trailer = 块尾追加的那一节 (机械臂自报负载); 空串 = 不追加
    // 正文为空也照写 (仍然留下"这次跑过"的痕迹, 且围栏保持成对)。
    // 正文末尾没有换行时【补一个】: 否则收尾的 ``` 会粘在最后一行后面, 围栏就不成围栏了。
    // 补的是正文【之后】的字节, 正文本身一个字节都没动。
    inline std::string block(const char* ts, int poses, int pairs,
                             const std::string& body,
                             const std::string& trailer = std::string()) {
        std::string s = blockHeader(ts, poses, pairs);
        s += "\n\n```text\n";
        s += body;
        if (body.empty() || body[body.size() - 1] != '\n') s += "\n";
        s += "```\n";
        s += trailer;
        return s;
    }

    // ===== 块尾的 d 判据那一节 (纯函数) =====
    //
    // d = cz_robot − c_s_z 与判据 (0, 31.5) mm (设计 §6b: 测量原点必须落在传感器体内)。
    //
    // 【为什么两个值都要打】: c_s 的 z 向【符号】不是数据定得下来的。模型
    //     F = b_F + A·g,   M = b_M + c_s × (A·g)
    // 在 g → −g, A → −A, c_s → −c_s 下【逐字不变】, 而 A 是自由 3×3 (含反射): parity =
    // sign(det A) 只把"含不含反射"报出来, 说不了哪一支是物理的。同一份 cz_robot 与 |c_s_z|
    // 于是给出两个 d, 而 (0, 31.5) mm 对这两个数的结论可以【相反】—— run-001 的实数就是
    // 13.144 mm (在范围内) 与 124.256 mm (在范围外)。
    // ⚠ 反向的约定【不是】"把 d 整体变负"—— 两支差 2·c_s_z, 谁偏大、谁落到零以下都随 c_s_z
    //   的符号翻。这里引的例子 (cz_robot = +68.7, c_s_z = +55.556) 算出 68.700 + 55.556 =
    //   124.256 mm, 恰好仍是正数; 换成 c_s_z = −100 就是 68.700 + (−100) = −31.3 mm —— 负数
    //   照样会出现在这一支上。把它一般化说成"另一个正数"会让操作员去错的地方找问题。
    //
    // 【所以这一节【不】给单一的 ✓/✗】: 上面两行谁也不是"通过", 读者拿不到一个能被单独引用的
    // 结论 —— 符号约定未定之前, 这里给不出"通过"。
    //
    // fitOk = 本次 fitRaw 的判决。为 false 时【照样打, 但把结论限定住】: 被拒的那一次, 块里
    // 几行之前才印着"模型形式检验 (fitRaw 的判决): 【拒绝】", 尾节再无条件地说一句"测量原点
    // 落在传感器体内", 就是同一个自描述块里两个互相打架的结论 (本项目最忌讳的"安静地错")。
    // 判据、阈值、任何判决路径一个字节没改 —— 加上的只是"这一行读不读得成立"的前提。
    //
    // ⚠ 这一节【只是信息】: 不参与任何接受/拒绝, 也不改变 fitRaw 的判决 (brief 硬要求 6)。
    //   两个约定的【成对读法】属于上机操作单 §6 闸1 (它管的是"要不要下发 cz", 不是本次
    //   拟合成不成) —— 这里只把两个数摆出来, 不替读者裁定。
    // ===== d 的两支 (约定一 / 约定二) —— 全项目【只此一份】实现 =====
    //
    // 定义与判据 (0, 31.5) mm 见上面那一段; 这里只把两个数抽出来, 供【两个】调用方共用:
    //   · 文档块那一节 (payloadDSection);
    //   · 下发前的【闸 1】(PayloadCalibration::evaluateSendGate, Task 8a)。
    // 两处各算一遍就是"同一个量两个实现" —— 本项目栽过的正是这种 (漂移之后两边各说各话,
    // 而且都在自己的上下文里看起来是对的)。
    //
    // 判据是【开】区间: d 恰好等于 0 或 31.5 都算不在内 (测量原点【在】传感器体内, 不是压在
    // 两端上)。
    // 判据的两端 (mm) —— 【开】区间。操作单 §6 闸1 写的是 "0 < d < 31.5" (31.5 = 传感器总高:
    // 测量原点必须落在传感器体内)。公开成常量是为了让打印端能【引用】它, 而不是在提示文字里
    // 再抄一遍 31.5 —— 抄一遍就是一个会在改判据时撒谎的第二来源。
    static const double PAYLOAD_D_MIN_MM = 0.0;
    static const double PAYLOAD_D_MAX_MM = 31.5;

    struct PayloadDValues {
        double sameDir;   // 约定一 (c_s_z 与工具轴同向): d = cz_robot − c_s_z  (mm)
        double flipDir;   // 约定二 (反向):               d = cz_robot + c_s_z  (mm)
        bool   sameIn;    // sameDir 落在 (PAYLOAD_D_MIN_MM, PAYLOAD_D_MAX_MM)
        bool   flipIn;    // flipDir 同上
    };
    inline PayloadDValues payloadDValues(double czRobotMm, double csZmm) {
        PayloadDValues v;
        v.sameDir = czRobotMm - csZmm;
        v.flipDir = czRobotMm + csZmm;
        v.sameIn  = (v.sameDir > PAYLOAD_D_MIN_MM && v.sameDir < PAYLOAD_D_MAX_MM);
        v.flipIn  = (v.flipDir > PAYLOAD_D_MIN_MM && v.flipDir < PAYLOAD_D_MAX_MM);
        return v;
    }

    inline std::string payloadDSection(double czRobotMm, double csZmm, bool fitOk) {
        // 约定一 = c_s_z 与工具轴【同向】 (改动前的实现取的就是这一支); 约定二 = 反向。
        // 两支的算术与判据不在这里写第二遍 —— 取上面那一份共用的实现。
        const PayloadDValues dv = payloadDValues(czRobotMm, csZmm);
        const double dSameDir = dv.sameDir;
        const double dFlipDir = dv.flipDir;
        const bool sameIn  = dv.sameIn;
        const bool flipIn  = dv.flipIn;

        std::string s;
        char buf[512];
        snprintf(buf, sizeof(buf),
                 "- d = cz_robot − c_s_z, 【两种符号约定都算】= %.17g / %.17g mm\n",
                 dSameDir, dFlipDir);
        s += buf;
        s += "  (c_s 的 z 向【符号】不由数据决定: 模型在 g → −g, A → −A, c_s → −c_s 下逐字不变。\n"
             "   两支差 2·c_s_z, 即【约定二 = 约定一 + 2·c_s_z】: c_s_z 为正时约定二偏大、为负时\n"
             "   约定一偏大, 谁落到零以下也跟着翻 (负的 c_s_z 下落到零以下的正是【反向】这一支)。\n"
             "   所以这里【不】给单一的勾/叉 —— 整节一个勾都不出现, 谁也别想从块里摘出一个\n"
             "   【通过】去。)\n";
        // ⚠ 两个标签里的符号【由数字自己带出来】, 不许在格式串里写死 "+"/"−":
        //   cS 是法方程解出来的【原始解】, 没有做任何符号归一 (force/PayloadCalibration.cpp 的
        //   cS 直接来自求解), 而这一节的论点恰恰是【符号不由数据定】—— 所以负的 c_s_z 与 run-001
        //   的正值【一样可能】。写死的符号 + 负值会印出 "c_s_z = +-55.556…" 与 "c_s_z = −-55.556…":
        //   标签与紧挨着的那个数【互相打架】, 而且正好发生在那段论证"符号未定"的文字里面。
        //   约定一 用 c_s_z 本身, 约定二 用它取负 (反向就是这一个数 —— 不是一个整体变号的 d);
        //   两个约定【都印出那个约定的真值】, 谁也不加修饰。约定的区别(同向/反向)是真的, 标签保留。
        snprintf(buf, sizeof(buf),
                 "    约定一【同向, c_s_z = %.17g mm】: d = %.17g − %.17g = %.17g mm  %s\n",
                 csZmm, czRobotMm, csZmm, dSameDir, sameIn ? "【在范围内】" : "【在范围外】");
        s += buf;
        snprintf(buf, sizeof(buf),
                 "    约定二【反向, c_s_z = %.17g mm】: d = %.17g + %.17g = %.17g mm  %s\n",
                 -csZmm, czRobotMm, csZmm, dFlipDir, flipIn ? "【在范围内】" : "【在范围外】");
        s += buf;
        s += "  → 判据 0 < d < 31.5 mm: 两种约定下结论";
        s += (sameIn == flipIn) ? "相同" : "【相反】";
        // 【不给单一的勾, 也不替读者裁定】: 这一条是【成对】读的 (上机操作单 §6 闸1:
        // 恰好一支落在内 = 那就是正确的符号约定 / 两支都在内 = 符号定不了, 不许猜 /
        // 两支都在外 = 哪里错了)。本块只把两个数摆出来, 谁也别想从块里摘走一支当"通过"。
        s += " —— 这一条要【成对读】(用法见上机操作单 §6 闸1): 本块【不给单一的勾】,\n"
             "    别只认其中一支。\n";
        if (!fitOk)
            s += "    （本行仅描述 d 的算术位置; 本次 fitRaw 为【拒绝】, 该结论不成立）\n";
        return s;
    }

    // ===== 追加写 =====

    // 把一段字节【追加】到文件末尾。返回 false = 没写成 (打不开 / 写不全), 调用方据此照实报。
    // 【只用 "a+" 打开, 永不截断。】不要先试 "r" 再决定 "w"/"a": "读不了"(被占用/权限) 与
    // "不存在"会被混为一谈, 而前者会走 "w" 把历次记录整个删掉 —— 毁的正是这个文件存在的理由。
    // 这条与 main.cpp 的 logCalibAttempt / logPoseData 是同一条规矩, 同样【没有例外】。
    inline bool appendToFile(const char* path, const std::string& text) {
        FILE* f = fopen(path, "a+");
        if (!f) return false;
#ifdef _WIN32
        // ===== 写【二进制】, 与上面那个 "a+" 不冲突 =====
        // "a+" 的文本模式会把每个 \n 翻成 \r\n —— 那么"文档里那一块"与"屏幕上的那一屏"就不再
        // 逐字节相同了 (而逐字节相同正是本任务要验收的东西), 文档也跟着变成 CRLF (本仓库的
        // .md 一律是 LF: Docs/superpowers/specs/*.md 里没有一个 \r)。
        // 追加语义只由打开方式决定 ("a+" = 只追加、永不截断), 与是不是二进制无关 —— 所以这里
        // 保持 "a+", 再把流的翻译关掉: 落盘的字节就是交给它的字节。
        _setmode(_fileno(f), _O_BINARY);
#endif
        const size_t n = text.size();
        const bool ok = (n == 0) || (fwrite(text.data(), 1, n, f) == n);
        fclose(f);
        return ok;
    }

    // ===== stderr 捕获窗口 =====
    //
    // 库 (PayloadCalibration::fitRaw) 把逐姿态残差表与 [Payload] 自检行打到 stderr —— 那是
    // "为什么被拒"的唯一出处, 而它【不经过调用方的打印】, 常规的 sink 收不到。
    // 做法: 调用前后各一次 _dup2, 让 fd 2 在窗口内指向一个临时文件, 收完再整段并回 sink。
    // (与 tests\test_payload_calibration.cpp 里"把 stderr 静音"是同一套办法。)
    //
    // 【还原是无条件的】: savedFd 是 _dup 出来的私有句柄, 只要它有效就一定还原得回去。
    // "把 stderr 弄丢"是本项目最不能接受的一类失败 (诊断被安静地藏起来), 所以:
    //   · Begin 的每一步失败都【什么都没改】就返回 false —— 这时 stderr 一个字节都不动,
    //     调用方照常往下走, 只在块尾照实说一句"这一段没并进来" (不许静默省略);
    //   · End 先还原再读文件, 读/删失败都不影响还原。
    //
    // ⚠ 这不是线程安全的设计: 窗口内别的线程若写 stderr, 它的字节也会落进临时文件 (随后被
    //   一并打进 stdout 与文档块 —— 屏幕不丢, 只是这行会出现在块里)。本窗口只有 fitRaw 一次
    //   纯计算, 时长短到无需为它上锁。
    struct StderrCaptureState {
        int  savedFd = -1;
        // 本次窗口【真正】用的临时文件路径 —— 不是调用方给的那个基名, 而是它的一个本次专属的
        // 变体 (见 captureTmpPathFor)。留给操作员的路径 (leftoverPath) 取的是它。
        char tmpPath[512] = {0};
        // 上一次 End 【读不出来】因而【故意没删】的那个临时文件 (空 = 没有)。
        // 它不是"没搭起窗口"(那时根本没有文件), 而是"字节已经写进去了、只是收不回来" ——
        // 那些字节的【唯一副本】就在这个文件里, 所以调用方必须把路径报出去 (见 stderrCaptureEnd)。
        char leftoverPath[512] = {0};
    };
    inline StderrCaptureState& stderrCaptureState() {
        static StderrCaptureState s;
        return s;
    }
    inline bool stderrCaptureActive() { return stderrCaptureState().savedFd >= 0; }

    // 上一次 stderrCaptureEnd 读失败时留下来的临时文件路径 (空串 = 没有)。
    // 只在那个失败的场合非空; 报给操作员, 那一段诊断还能被人从盘上捡回来。
    inline const char* stderrCaptureLeftoverPath() { return stderrCaptureState().leftoverPath; }

    // ===== 本次捕获【专属】的临时文件名 (F2) =====
    //
    // 为什么【必须每次都不一样】: 读失败时那个临时文件是【故意不删】的 —— 它此刻是那些字节的
    // 唯一副本 (窗口里 fd 2 指着它, 所以它们压根没进控制台), 而它的路径刚被写进块尾交给操作员
    // ("原始字节没有丢, 它们还在临时文件里: …")。可见的操作员最可能做的下一件事就是【再按一次
    // 's' 看一遍】。而这个名字从前是【固定的】(calib_stderr.tmp), Begin 又是 _O_CREAT|_O_TRUNC
    // —— 于是那一次按键【当场把刚刚指着的那份副本截成 0 字节】, 控制台与文档块两头都没有了。
    //
    // 做法: 真正的文件名 = 基名 + ".<pid>_<序号>" + 扩展名 (序号每次 Begin 递增), 再配合
    // _O_EXCL (绝不打开已存在的文件, 见 stderrCaptureBegin) —— 残留因此不可能被后一次抹掉。
    // 基名里【没有 '.'】(或 '.' 只出现在目录名里) 时, 后缀直接接在末尾。
    inline unsigned long& captureSeqCounter() { static unsigned long s = 0; return s; }

    inline std::string captureTmpPathFor(const char* base, unsigned long pid, unsigned long seq) {
        std::string s = (base && *base) ? base : "capture.tmp";
        const std::string tag = "." + std::to_string(pid) + "_" + std::to_string(seq);
        const size_t slash = s.find_last_of("/\\");
        const size_t dot   = s.find_last_of('.');
        if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
            s.insert(dot, tag);            // 插在末一个 '.' 之前: 扩展名还认得出
        else
            s += tag;                      // 没有扩展名 (目录里的 '.' 不算) -> 接在末尾
        return s;
    }

    // 开始捕获。tmpPath = 临时文件的【基名】(调用方给, 通常是 CalibStore::fileFor(...)) ——
    // 真正建出来的是它一个【本次专属】的变体 (见 captureTmpPathFor), 捕获成功后会被删掉;
    // 读失败时不删, 那条路径由 stderrCaptureLeftoverPath() 报出去 (报的【就是】盘上那个文件)。
    inline bool stderrCaptureBegin(const char* tmpPath) {
        StderrCaptureState& st = stderrCaptureState();
        if (tmpPath == nullptr || *tmpPath == '\0') return false;
        if (st.savedFd >= 0) return false;                 // 已经在窗口里了, 不套娃
        if (strlen(tmpPath) >= sizeof(st.tmpPath)) return false;

#ifdef _WIN32
        fflush(stderr);                                    // 窗口之前已经在缓冲里的字节, 不进窗口
        // ===== 造一个【不碰任何已存在文件】的名字 =====
        // _O_EXCL 而不是 _O_TRUNC: 候选名已经被占着 (上一次的残留 / pid 被复用 / 盘上本来就有
        // 个同名文件) 时 _open 以 EEXIST 失败 —— 那就【换下一个序号重试】, 一个字节都不动它。
        // 于是"后一次 's' 把前一次残留的副本截掉"这件事在文件系统这一层就不可能发生。
        // 试满 64 个还不行就【什么都不改地失败】(调用方照实报"窗口没搭起来"), 不猜、不覆盖。
        int fd = -1;
        for (int attempt = 0; attempt < 64; attempt++) {
            const std::string cand = captureTmpPathFor(tmpPath, (unsigned long)_getpid(),
                                                      ++captureSeqCounter());
            if (cand.size() >= sizeof(st.tmpPath)) break;   // 放不下 -> 不做半个路径的猜测
            fd = _open(cand.c_str(), _O_CREAT | _O_EXCL | _O_WRONLY | _O_BINARY,
                       _S_IREAD | _S_IWRITE);
            if (fd >= 0) {
                snprintf(st.tmpPath, sizeof(st.tmpPath), "%s", cand.c_str());
                break;
            }
            if (errno != EEXIST) break;                     // 不是撞名 (目录不可写…) -> 重试没意义
        }
        if (fd < 0) return false;
        const int saved = _dup(2);
        if (saved < 0) { _close(fd); return false; }
        if (_dup2(fd, 2) != 0) { _close(saved); _close(fd); return false; }
        // fd 2 已指向同一个打开文件, 这个多余的句柄可以关掉 (文件因 fd 2 仍然开着)
        _close(fd);
        st.savedFd = saved;
        // (st.tmpPath 在上面建文件的时候就填好了 —— 它是【真正】建出来那个名字, 不是基名。
        //  报给操作员的路径必须与盘上那个文件一致, 所以这里绝不能再写回基名。)
        return true;
#else
        (void)tmpPath;
        return false;
#endif
    }

    // 结束捕获: 【无条件还原 fd 2】, 并把窗口内收到的字节追加到 out (out 可为 nullptr = 只要还原)。
    // 返回 false = 窗口里收到了内容但【一个字都没并进 out】—— 打不开 (fd < 0) 或读到一半失败
    // (n < 0, 见下) 都算; 两种情况下调用方都照实报, 并拿 stderrCaptureLeftoverPath() 去捡字节。
    //
    // 【读不出来时【不删】那个临时文件】: 上面那一句 fflush(stderr) 已经把 stderr 缓冲里的字节
    // 推进临时文件了 (fd 2 在窗口里就指着它), 所以此刻盘上那份【是这些字节唯一的副本】——
    // 删掉它 = 控制台与文档块两头都没有了, 只留下一句"收回来了但读不出来", 正是本文件开头说的
    // "本项目最不能接受的一类失败"。留着, 把路径交给 stderrCaptureLeftoverPath(), 至少还能捡回来。
    // (读成功时照旧删掉: 字节已经在 out 里了。)
    inline bool stderrCaptureEnd(std::string* out) {
        StderrCaptureState& st = stderrCaptureState();
        if (st.savedFd < 0) return false;
#ifdef _WIN32
        fflush(stderr);                                    // 把 stderr 缓冲里剩下的字节推进临时文件
        const int saved = st.savedFd;
        st.savedFd = -1;
        _dup2(saved, 2);                                   // ← 先还原, 后面无论如何都在窗口外了
        _close(saved);

        st.leftoverPath[0] = '\0';
        bool ok = true;
        // 本调用【之前】out 有多长: 中途读失败时, 本调用已经追加进去的那半截要退回去 (见下)。
        const size_t outLen0 = out ? out->size() : 0;
        const int fd = _open(st.tmpPath, _O_RDONLY | _O_BINARY);
        if (fd < 0) {
            ok = false;
            // 【不删】, 只把路径记下来 —— 见上面那段注释。
            snprintf(st.leftoverPath, sizeof(st.leftoverPath), "%s", st.tmpPath);
        } else {
            char buf[4096];
            int n = 0;
            while ((n = _read(fd, buf, (unsigned)sizeof(buf))) > 0) {
                if (out) out->append(buf, (size_t)n);
            }
            _close(fd);
            // 【_read 返回负数 ≠ EOF】: `while (... > 0)` 分不出"读完了"与"读坏了"(I/O 错误 /
            // 杀软正占着这个文件)。从前读坏了也一路走到 _unlink: out 里是【半截】、ok 却是 true,
            // 调用方把它当成整段发出去 (没有警告), 临时文件还被删掉 —— 缺的尾巴【安静地没了】。
            // 那正是本文件开头说的"诊断被安静地藏起来", 藏的只是尾巴而已。
            // 所以与 _open 失败【同等对待】: 照实返回 false / 文件【不删】/ 路径报出去。
            if (n < 0) {
                ok = false;
                // 半截不许发出去 —— 调用方那句"这一段没能并入本块"必须是真的 (与 _open 失败时
                // 一模一样: 一个字都不并入)。字节没丢, 它们在【不删】的那个文件里, 路径照报。
                if (out) out->resize(outLen0);
                snprintf(st.leftoverPath, sizeof(st.leftoverPath), "%s", st.tmpPath);
            } else {
                _unlink(st.tmpPath);
            }
        }
        st.tmpPath[0] = '\0';
        return ok;
#else
        (void)out;
        return false;
#endif
    }
}
