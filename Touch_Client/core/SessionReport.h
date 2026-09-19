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
#ifdef _WIN32
#include <io.h>        // _dup / _dup2 / _close / _open / _read / _unlink
#include <fcntl.h>     // _O_CREAT / _O_TRUNC / _O_WRONLY / _O_RDONLY / _O_BINARY
#include <sys/stat.h>  // _S_IREAD / _S_IWRITE
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
        char tmpPath[512] = {0};
    };
    inline StderrCaptureState& stderrCaptureState() {
        static StderrCaptureState s;
        return s;
    }
    inline bool stderrCaptureActive() { return stderrCaptureState().savedFd >= 0; }

    // 开始捕获。tmpPath = 临时文件路径 (调用方给; 捕获结束后会被删掉)。
    inline bool stderrCaptureBegin(const char* tmpPath) {
        StderrCaptureState& st = stderrCaptureState();
        if (tmpPath == nullptr || *tmpPath == '\0') return false;
        if (st.savedFd >= 0) return false;                 // 已经在窗口里了, 不套娃
        if (strlen(tmpPath) >= sizeof(st.tmpPath)) return false;

#ifdef _WIN32
        fflush(stderr);                                    // 窗口之前已经在缓冲里的字节, 不进窗口
        const int fd = _open(tmpPath, _O_CREAT | _O_TRUNC | _O_WRONLY | _O_BINARY,
                             _S_IREAD | _S_IWRITE);
        if (fd < 0) return false;
        const int saved = _dup(2);
        if (saved < 0) { _close(fd); return false; }
        if (_dup2(fd, 2) != 0) { _close(saved); _close(fd); return false; }
        // fd 2 已指向同一个打开文件, 这个多余的句柄可以关掉 (文件因 fd 2 仍然开着)
        _close(fd);
        st.savedFd = saved;
        snprintf(st.tmpPath, sizeof(st.tmpPath), "%s", tmpPath);
        return true;
#else
        (void)tmpPath;
        return false;
#endif
    }

    // 结束捕获: 【无条件还原 fd 2】, 并把窗口内收到的字节追加到 out (out 可为 nullptr = 只要还原)。
    // 返回 false = 窗口里收到了内容但读不出来 (调用方据此照实报)。
    inline bool stderrCaptureEnd(std::string* out) {
        StderrCaptureState& st = stderrCaptureState();
        if (st.savedFd < 0) return false;
#ifdef _WIN32
        fflush(stderr);                                    // 把 stderr 缓冲里剩下的字节推进临时文件
        const int saved = st.savedFd;
        st.savedFd = -1;
        _dup2(saved, 2);                                   // ← 先还原, 后面无论如何都在窗口外了
        _close(saved);

        bool ok = true;
        const int fd = _open(st.tmpPath, _O_RDONLY | _O_BINARY);
        if (fd < 0) {
            ok = false;
        } else {
            char buf[4096];
            int n;
            while ((n = _read(fd, buf, (unsigned)sizeof(buf))) > 0) {
                if (out) out->append(buf, (size_t)n);
            }
            _close(fd);
        }
        _unlink(st.tmpPath);
        st.tmpPath[0] = '\0';
        return ok;
#else
        (void)out;
        return false;
#endif
    }
}
