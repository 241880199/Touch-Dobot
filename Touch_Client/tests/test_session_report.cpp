// Standalone test: SessionReport — 上机整屏落成 calib\calib_report.md
// Build: build_session_report_test.bat
// Run:   test_session_report.exe
//
// 【这条测试为什么能当"同源"的验收】: 屏幕那一半 (main.cpp 的 BiasCheck::diagEmit) 与块里的
// 正文是【同一个 std::string】—— 本测试把"块把一段给定的诊断文本逐字包住、一个字节都不动、
// 且块头格式正确"钉死; 剩下的一半 (那段文本确实就是屏幕上的字节) 由 diagEmit 的结构保证
// (它把同一份字节 fwrite 给 stdout 又 append 给正文, 没有第二条打印路径)。
//
// 另外两条必须有单测的 (没有实机也要能验):
//   · 追加语义: 已有内容一个字节不动 —— 尤其"文件读不了"时【不得】退化成截断;
//   · stderr 捕获窗口: 窗口内的字节能收回来, 且【窗口结束后 fd 2 必须还原】(诊断不许被弄丢)。

#include <cstdio>
#include <cstring>
#include <string>
#include <cerrno>     // errno / EIO
#include <direct.h>   // _mkdir / _chdir / _getcwd
#include <io.h>       // _access / _unlink / _rmdir (+ 真的 _read, 先取到手)
#include <sys/stat.h> // _chmod / _S_IREAD / _S_IWRITE

// ===== 注错点: 让 SessionReport.h 里那一句 _read 失败 =====
// 服务的是 test_stderr_capture_end_read_error_is_a_failure_not_eof (N4)。
// 【为什么非注错不可】: Windows 上"打开了却读不出来"造不出来 —— 目录在 _open 那一步就被挡掉
// (EACCES, 实测), 只读文件照样读得出来。所以"中途读失败"这条路【从前没有任何测试走到过】,
// 而它正是本次修的那一条: `while (_read(...) > 0)` 分不出"读完了"与"读坏了"(I/O 错误 / 杀软
// 正占着这个文件), 于是磁盘上那份的【半截】被当成整段发出去 (没有警告), 临时文件还被删掉。
// 做法: 先取到真的 _read, 再用宏把头文件里出现的 _read 改名到下面的转发函数 ——
// 【注错只在本 TU 生效, 产品代码一个字节没动】。计数器与字节数保证"这条路真的被走到了",
// 而不是"在旁边断言一句理论上会失败"。
using ReadFn = int (*)(int, void*, unsigned);
static ReadFn g_realRead = &::_read;    // ← 必须在 #define 之前取
static bool   g_failRead = false;       // 置位后: 读先给半截, 然后一律失败
static int    g_partialFirstRead = 0;   // >0 = 第一次最多给这么多字节; <0 = 已经给过 -> 之后一律 -1
static int    g_fakeReadCalls = 0;      // 注错点被走到几次
static int    g_fakeReadErrors = 0;     // 其中返回 -1 (读坏了) 几次
static long   g_fakeBytesServed = 0;    // 注错点一共交出去多少字节 (证明"半截"真的进过 out)
static int fakeRead(int fd, void* buf, unsigned n) {
    g_fakeReadCalls++;
    if (!g_failRead) return g_realRead(fd, buf, n);
    if (g_partialFirstRead < 0) { g_fakeReadErrors++; errno = EIO; return -1; }
    if ((unsigned)g_partialFirstRead < n) n = (unsigned)g_partialFirstRead;
    const int got = g_realRead(fd, buf, n);
    g_partialFirstRead = -1;                // 这一口已经给出去, 下一次起一律失败
    if (got > 0) g_fakeBytesServed += got;
    return got;
}
#define _read fakeRead
#include "../core/SessionReport.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::printf("  " #name "... "); } while(0)
#define PASS() do { std::printf("PASS\n"); g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::printf("FAIL: %s\n", #cond); g_failed++; return; } } while(0)

// 测试自己的临时目录 (与 test_calibstore_tmp 同一个办法: 不往 tests/ 根上撒文件)
static const char* TMPDIR = "test_session_report_tmp";

static std::string tmpPath(const char* name) {
    std::string s = TMPDIR;
    s += "/";
    s += name;
    return s;
}

static bool writeWholeFile(const std::string& path, const std::string& text) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return false;
    const bool ok = (fwrite(text.data(), 1, text.size(), f) == text.size());
    fclose(f);
    return ok;
}

static std::string readWholeFile(const std::string& path) {
    std::string out;
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return out;
    char buf[4096];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    fclose(f);
    return out;
}

static const char* TS = "2026-09-19 15:33:38";

// 一段"像真的"的诊断正文: 多行、含中文、含 % 与数字、以换行结尾
static std::string sampleBody() {
    return std::string(
        "  原始通道 (@1304 SixForceValue) 线性解 — 10 个姿态\n"
        "  模型:  F = b_F + A·g        M = b_M + c_s × (A·g)\n"
        "  c_s (质心, 【传感器测量系】)              = (-0.244, +0.349, +55.556) mm\n"
        "  拟合残差  rmsForceN                       = 0.022400 N\n"
        "  模型形式检验 (fitRaw 的判决): 【拒绝】\n"
        "      力通道:   残差÷尺子 χ²/dof = 4.2  >=  门限 3.1   (dof=18, 尺子 0.0110 N)\n");
}

// ===== 块格式 =====

static void test_block_header_format() {
    TEST(block_header_format);
    CHECK(SessionReport::blockHeader(TS, 10, 5) == "## 采集 2026-09-19 15:33:38 — 10 姿态 / 5 对");
    // 0 对 (没按 'r') 也要如实写出来 —— 那正是最要紧的一次判决的输入
    CHECK(SessionReport::blockHeader(TS, 3, 0) == "## 采集 2026-09-19 15:33:38 — 3 姿态 / 0 对");
    // 时间戳缺失时不许崩、也不许写出半截块头
    CHECK(SessionReport::blockHeader(nullptr, 1, 0).find("姿态 / 0 对") != std::string::npos);
    PASS();
}

// 【核心】正文逐字: 位置与字节都要对得上
static void test_block_wraps_body_verbatim() {
    TEST(block_wraps_body_verbatim);
    const std::string body = sampleBody();
    const std::string blk = SessionReport::block(TS, 10, 5, body, "");

    const std::string head = std::string("## 采集 ") + TS + " — 10 姿态 / 5 对\n\n```text\n";
    CHECK(blk.compare(0, head.size(), head) == 0);                  // 块头 + 空行 + 围栏
    CHECK(blk.compare(head.size(), body.size(), body) == 0);        // 正文【逐字】就在这个位置
    CHECK(blk.compare(head.size() + body.size(), 4, "```\n") == 0); // 收尾围栏
    CHECK(blk.size() == head.size() + body.size() + 4);             // 一个字节都不多
    PASS();
}

// 正文末尾没有换行时, 收尾围栏仍必须自成一行 (否则围栏就不是围栏)
// —— 补的字节在正文【之后】, 正文本身仍是逐字
static void test_block_closes_fence_when_body_has_no_trailing_newline() {
    TEST(block_closes_fence_when_body_has_no_trailing_newline);
    const std::string body = "(空屏)";  // 没有结尾换行
    const std::string blk = SessionReport::block(TS, 2, 0, body, "");
    const std::string head = std::string("## 采集 ") + TS + " — 2 姿态 / 0 对\n\n```text\n";
    CHECK(blk.compare(head.size(), body.size(), body) == 0);
    CHECK(blk.compare(head.size() + body.size(), 5, "\n```\n") == 0);

    // 正文为空也照写: 仍然留下"这次跑过"的痕迹, 且围栏成对
    const std::string empty = SessionReport::block(TS, 0, 0, "");
    CHECK(empty.find("```text\n\n```\n") != std::string::npos);
    PASS();
}

static void test_block_trailer_goes_after_the_fence() {
    TEST(block_trailer_goes_after_the_fence);
    const std::string body = sampleBody();
    const std::string trailer =
        "\n### 机械臂自报负载 (30004 帧)\n"
        "- @1168 Load        = 0.40400000000000003 kg\n"
        "- d = cz_robot − c_s_z = 68.699999999999989 − 55.556000000000004 = 13.143999999999996 mm\n"
        "- ⚠ 本节只是信息: 不参与任何接受/拒绝, 也不改变 fitRaw 的判决。\n";
    const std::string blk = SessionReport::block(TS, 10, 5, body, trailer);
    const std::string head = std::string("## 采集 ") + TS + " — 10 姿态 / 5 对\n\n```text\n";
    CHECK(blk.compare(head.size(), body.size(), body) == 0);
    CHECK(blk.compare(head.size() + body.size(), 4, "```\n") == 0);
    CHECK(blk.compare(head.size() + body.size() + 4, trailer.size(), trailer) == 0);
    // 尾节【在围栏之外】—— 它不是"控制台输出"的一部分, 别混进正文里
    // (head 以 "```text\n" 收尾, 所以第一个 "```" 正好落在 head.size()-8)
    CHECK(blk.find("```") == head.size() - 8);
    PASS();
}

// 【真东西】: 2026-09-19 那次上机(15:33, 9 姿态 / 3 对)控制台上的那一屏 —— 从
// Docs/superpowers/specs/2026-09-19-raw-channel-calibration-run-001.md 的"附: 原始控制台
// 输出"逐字取下来的夹具 (那时这一屏还是人肉转录的, 正是本次要取代的那件事)。
// 它比上面那段合成正文更硬: 里面同时有主程序打到 stdout 的块【和】库打到 stderr 的
// [Payload] 逐姿态残差表, 顺序就是屏幕上看到的那个顺序 (夹具缺了 = 检出缺文件, 不给 SKIP)。
static void test_block_wraps_a_real_console_screen_verbatim() {
    TEST(block_wraps_a_real_console_screen_verbatim);
    const std::string screen = readWholeFile("fixtures/console_screen_2026-09-19_run001.txt");
    CHECK(!screen.empty());
    CHECK(screen.find("[Payload] 逐姿态残差") != std::string::npos);   // 真夹具, 不是空文件

    const std::string trailer =
        "\n### 机械臂自报负载 (30004 帧)\n"
        "- @1168 Load        = 0.40400000000000003 kg\n"
        "- d = cz_robot − c_s_z = 68.699999999999989 − 55.555999999999997 = 13.144000000000002 mm\n"
        "- ⚠ 本节只是信息: 不参与任何接受/拒绝, 也不改变 fitRaw 的判决。\n";
    const std::string blk = SessionReport::block("2026-09-19 15:33:38", 9, 3, screen, trailer);

    // 屏幕上那一屏【逐字节】在块里, 且块头/围栏/尾节各就各位
    const std::string head = "## 采集 2026-09-19 15:33:38 — 9 姿态 / 3 对\n\n```text\n";
    CHECK(blk.compare(0, head.size(), head) == 0);
    CHECK(blk.compare(head.size(), screen.size(), screen) == 0);
    CHECK(blk.compare(head.size() + screen.size(), 4, "```\n") == 0);
    CHECK(blk.compare(head.size() + screen.size() + 4, trailer.size(), trailer) == 0);
    CHECK(blk.size() == head.size() + screen.size() + 4 + trailer.size());
    PASS();
}

// ===== 块尾 d 判据: 【两种符号约定都算】, 且不给单一的 ✓/✗ =====
//
// c_s 的 z 向【符号】不由数据决定 (模型在 g → −g, A → −A, c_s → −c_s 下逐字不变), 所以同一个
// cz_robot 与 |c_s_z| 给出两个 d, 而 (0, 31.5) mm 对它们的结论可以【相反】。用 run-001 的实数:
//   同向  68.700 − 55.556 =  13.144 mm  -> 在范围内
//   反向  68.700 + 55.556 = 124.256 mm  -> 在范围外
// 从前这里只打一支, 而且块里的句子说"若实际反向, d 会整体变负" —— 那是算错的。真正的说法是
// 【两支差 2·c_s_z】: 本例 c_s_z > 0, 所以反向那支偏大且仍是正数; c_s_z < 0 时偏大的是同向那支、
// 落到零以下的才是反向那支 —— "反向给的是另一个正数"这种一般化的说法只在 c_s_z > 0 时真。
// 这个用例把"两支都打 + 不给单一结论"钉住。
static void test_payload_d_section_prints_both_sign_conventions() {
    TEST(payload_d_section_prints_both_sign_conventions);
    // run-001 的自报值, 全精度 (取自那次上机的 30004 帧读数)
    const double czRobot = 68.699999999999989;   // @1176 CenterZ (mm)
    const double csZ     = 55.556000000000004;   // c_s 沿工具轴的分量 (mm)
    const std::string s = SessionReport::payloadDSection(czRobot, csZ, true);

    // 【两支的数值都在】—— 用 %.17g 逐字比 (文档是给人复算的, 精度不许缩水)
    char buf[128];
    snprintf(buf, sizeof(buf), "%.17g", czRobot - csZ); CHECK(s.find(buf) != std::string::npos);
    snprintf(buf, sizeof(buf), "%.17g", czRobot + csZ); CHECK(s.find(buf) != std::string::npos);

    // 两个结论【都】照实写出来 (这一对数是相反的), 并明说两种约定未必一致
    CHECK(s.find("【在范围内】") != std::string::npos);
    CHECK(s.find("【在范围外】") != std::string::npos);
    CHECK(s.find("【相反】") != std::string::npos);

    // 【不许有一个能被单独引用的 ✓】: 一个勾都不给, 而且明写要【成对读】、不给单一的勾
    CHECK(s.find("✓") == std::string::npos);
    CHECK(s.find("✗") == std::string::npos);
    CHECK(s.find("本块【不给单一的勾】") != std::string::npos);
    CHECK(s.find("成对读") != std::string::npos);

    // 【那句算错的话不许再出现】: 说"反向给的是另一个正数"是把本例 (c_s_z > 0) 当成了通例 ——
    //   c_s_z < 0 时负的正是反向那一支 (见下面那个负 c_s_z 的用例)。块里改成只说【两支的关系】。
    CHECK(s.find("变负") == std::string::npos);
    CHECK(s.find("另一个正数") == std::string::npos);
    CHECK(s.find("不是负数") == std::string::npos);
    CHECK(s.find("2·c_s_z") != std::string::npos);   // ← 关系在, 而且【关系】才是可搬走的真话
    snprintf(buf, sizeof(buf), "%.17g", czRobot + csZ);
    CHECK(czRobot + csZ > 0.0);            // 事实层面(仅本例): 反向那一支是正的 (run-001: 124.256 mm)
    CHECK(s.find(buf) != std::string::npos);

    // 缺 cz_robot 的那一支由 main.cpp 自己写 (【不可用】), 不在这里 —— 这里只管有数的时候。
    PASS();
}

// ===== 尾节对【被拒的那一次】要限定措辞 (I2) =====
// 被 chi2RepForceRatio 拒掉的那一次, 块里几行之前印着"模型形式检验: 【拒绝】"。尾节若无条件地
// 说一句"测量原点落在传感器体内", 同一个自描述块里就有了两个互相打架的结论 —— 正是本项目
// 最忌讳的"安静地错"。修法: 【限定】, 而不是删掉 (判据本身是信息, 该给的还得给)。
static void test_payload_d_section_qualifies_a_rejected_fit() {
    TEST(payload_d_section_qualifies_a_rejected_fit);
    const double czRobot = 68.699999999999989;
    const double csZ     = 55.556000000000004;

    const std::string rejected = SessionReport::payloadDSection(czRobot, csZ, false);
    CHECK(rejected.find("本次 fitRaw 为【拒绝】") != std::string::npos);
    CHECK(rejected.find("该结论不成立") != std::string::npos);
    CHECK(rejected.find("仅描述 d 的算术位置") != std::string::npos);
    // 【限定, 不是删掉】: 两支的数与两个结论一个都不能少
    CHECK(rejected.find("【在范围内】") != std::string::npos);
    CHECK(rejected.find("【在范围外】") != std::string::npos);
    CHECK(rejected.find("本块【不给单一的勾】") != std::string::npos);

    // fitOk = true 的那一次【不许】拖这条尾巴 (它会在块里自己打自己的脸)
    const std::string accepted = SessionReport::payloadDSection(czRobot, csZ, true);
    CHECK(accepted.find("该结论不成立") == std::string::npos);
    CHECK(accepted.find("【拒绝】") == std::string::npos);
    // 除了那一句限定, 两者逐字相同 —— 说明限定是【加上去的】, 没有动任何数
    CHECK(accepted.size() < rejected.size());
    CHECK(rejected.compare(0, accepted.size(), accepted) == 0);
    PASS();
}

// ===== 负的 c_s_z: 标签里的符号必须由【数字自己】带出来 (N2) =====
// cS 是法方程解出来的【原始解】, 没有做任何符号归一 (force/PayloadCalibration.cpp 的 cS 直接来自
// 求解) —— 所以负的 c_s_z 与 run-001 的正值【一样可能】; 而这一节的论点恰恰是"符号不由数据定"。
// 从前两个标签把 "+"/"−" 写死在格式串里, 于是这里会印出:
//     c_s_z = +-55.556…   /   c_s_z = −-55.556…
// —— 标签与紧挨着的那个数【互相打架】, 而且正好发生在那段论证"符号未定"的文字里面。
// 这个用例把"负值也不许出现拼出来的符号"钉住 (既有用例只喂了正值, 所以从来没人走过这一支)。
static void test_payload_d_section_negative_cs_prints_no_fabricated_signs() {
    TEST(payload_d_section_negative_cs_prints_no_fabricated_signs);
    const double czRobot = 68.699999999999989;
    const double csZ     = -55.556000000000004;   // ← 负的 |c_s_z|, 与 run-001 一样大
    const std::string s = SessionReport::payloadDSection(czRobot, csZ, true);

    // 【不许有拼出来的符号】: "+-" 与 "−-" 都是"把一个数说成它自己的相反数"
    CHECK(s.find("+-") == std::string::npos);
    CHECK(s.find("−-") == std::string::npos);

    // 【两个约定的真值都在】, 且各自带自己的符号 (约定二 = 约定一取负 —— 反向就是这么定义的)
    char buf[128];
    snprintf(buf, sizeof(buf), "= %.17g mm", csZ);   CHECK(s.find(buf) != std::string::npos);
    snprintf(buf, sizeof(buf), "= %.17g mm", -csZ);  CHECK(s.find(buf) != std::string::npos);
    // 两个标签本身还在 (它们区分的是【真事】: 同向/反向 —— 保留)
    CHECK(s.find("约定一【同向") != std::string::npos);
    CHECK(s.find("约定二【反向") != std::string::npos);

    // 两个 d 照旧【都算、都打】, 结论相反这件事照旧照说 (负的 c_s_z 下同样成立)
    snprintf(buf, sizeof(buf), "%.17g", czRobot - csZ); CHECK(s.find(buf) != std::string::npos);
    snprintf(buf, sizeof(buf), "%.17g", czRobot + csZ); CHECK(s.find(buf) != std::string::npos);
    CHECK(s.find("【相反】") != std::string::npos);

    // 正值那一支也要照同一条规矩 (别只在负值上打补丁: 正的 csZ 下同样不许有拼出来的符号)
    const std::string pos = SessionReport::payloadDSection(czRobot, -csZ, true);
    CHECK(pos.find("+-") == std::string::npos);
    CHECK(pos.find("−-") == std::string::npos);
    PASS();
}

// ===== 追加写: 已有内容一个字节不动, 永不截断 =====

static void test_append_preserves_existing_bytes() {
    TEST(append_preserves_existing_bytes);
    const std::string path = tmpPath("append.md");
    // 造一份"历次记录" (两个旧块)
    const std::string old1 = SessionReport::block("2026-09-19 15:25:00", 7, 3, "旧的一屏\n", "");
    const std::string old2 = SessionReport::block("2026-09-19 15:30:00", 8, 4, "旧的一屏二\n", "");
    CHECK(writeWholeFile(path, old1 + old2));

    const std::string blk = SessionReport::block(TS, 10, 5, sampleBody(), "");
    CHECK(SessionReport::appendToFile(path.c_str(), blk));

    const std::string after = readWholeFile(path);
    CHECK(after.size() == old1.size() + old2.size() + blk.size());
    CHECK(after.compare(0, old1.size(), old1) == 0);                    // 历次记录: 一个字节没动
    CHECK(after.compare(old1.size(), old2.size(), old2) == 0);
    CHECK(after.compare(old1.size() + old2.size(), blk.size(), blk) == 0); // 新块接在后面

    // 再追加一次: 只增不减
    CHECK(SessionReport::appendToFile(path.c_str(), blk));
    const std::string after2 = readWholeFile(path);
    CHECK(after2.size() == after.size() + blk.size());
    CHECK(after2.compare(0, after.size(), after) == 0);
    PASS();
}

static void test_append_creates_missing_file() {
    TEST(append_creates_missing_file);
    const std::string path = tmpPath("fresh.md");
    _unlink(path.c_str());
    const std::string blk = SessionReport::block(TS, 4, 0, "只有一行的正文\n", "");
    CHECK(SessionReport::appendToFile(path.c_str(), blk));
    CHECK(readWholeFile(path) == blk);
    // 【落盘的字节就是交给它的字节】: 文本模式会把 \n 翻成 \r\n, 那样"文档里那一块"与
    // "屏幕上的那一屏"就不再逐字节相同了 (而本仓库的 .md 一律是 LF)。这条把它钉死。
    CHECK(readWholeFile(path).find('\r') == std::string::npos);
    PASS();
}

// 【"只用 a+" 那条规矩要防的事故】: 文件【读不了】(被占用/只读) 时, 不能与"不存在"混为一谈
// 而走 "w" —— 那会把历次记录整个删掉。这里把它做成只读文件, 断言:
//   · appendToFile 如实返回 false (调用方据此照实报"没写成")
//   · 文件内容一个字节都没少
static void test_append_on_unreadable_file_reports_failure_without_truncating() {
    TEST(append_on_unreadable_file_reports_failure_without_truncating);
    const std::string path = tmpPath("readonly.md");
    const std::string old1 = SessionReport::block("2026-09-19 15:25:00", 7, 3, "旧的一屏\n", "");
    CHECK(writeWholeFile(path, old1));
    CHECK(_chmod(path.c_str(), _S_IREAD) == 0);   // 只读 (Windows 上 fopen "a+" 会失败)

    const bool ok = SessionReport::appendToFile(path.c_str(),
                                                SessionReport::block(TS, 10, 5, "新的一屏\n", ""));
    CHECK(!ok);                                    // 没写成 -> 调用方照实说, 不许静默
    CHECK(readWholeFile(path) == old1);            // ← 历次记录【一个字节都没少】

    CHECK(_chmod(path.c_str(), _S_IREAD | _S_IWRITE) == 0);  // 还原, 便于清理
    PASS();
}

// 【同源的另一半: 求解路径上不许有绕过 sink 的打印】
// "屏幕与文档逐字节同源"由两件事合成:
//   (1) 每一行【都】走 sink —— 这条是结构性断言, 能在没有实机时验, 就是下面这个测试;
//   (2) sink 把同一份字节同时交给 stdout 与正文 —— 那是 BiasCheck::diagEmit 的两行
//       (fwrite + append), 读一眼就成立, 没有第二条路径可走。
// 少了 (1), 任何一次"顺手加个 std::cout"都会让文档比屏幕少一行 —— 而那种漂移是安静的。
// 这个测试因此是【回归闸门】: 谁在求解路径里直接打印, 它就会红。
static void test_solve_path_prints_only_through_the_sink() {
    TEST(solve_path_prints_only_through_the_sink);
    const std::string src = readWholeFile("../main.cpp");
    CHECK(!src.empty());

    // 取 solveAndApply 的函数体: 从它的定义行到停用块之前 —— 用【那一行的原文】当标记 (不数括号,
    // 也不拿裸 "#if 0" 当标记: 注释里就出现过"下面的 #if 0"这种字样, 会切到半路)
    const size_t fn = src.find("static void solveAndApply() {");
    CHECK(fn != std::string::npos);
    const size_t stop = src.find("#if 0  // ===", fn);
    CHECK(stop != std::string::npos);
    const std::string body = src.substr(fn, stop - fn);
    CHECK(body.size() > 15000);   // 切出来的确实是那个函数体 (切错地方会小得离谱)

    // 这些一去文档就少一行的写法: 直接打印 / 直接写别的流
    CHECK(body.find("std::cout") == std::string::npos);
    CHECK(body.find("std::cerr") == std::string::npos);
    CHECK(body.find("fprintf(") == std::string::npos);
    CHECK(body.find("fputs(") == std::string::npos);
    CHECK(body.find("fwrite(") == std::string::npos);
    CHECK(body.find("puts(") == std::string::npos);
    CHECK(body.find("putchar(") == std::string::npos);
    // printf 要按"独立标识符"判: snprintf( 是允许的 (它只往缓冲里写, 不往屏幕上写)
    for (size_t i = body.find("printf("); i != std::string::npos; i = body.find("printf(", i + 1)) {
        const char pre = (i > 0) ? body[i - 1] : ' ';
        const bool standalone = !((pre >= 'a' && pre <= 'z') || (pre >= 'A' && pre <= 'Z')
                               || (pre >= '0' && pre <= '9') || pre == '_');
        CHECK(!standalone);   // 独立的 printf( = 绕过了 sink
    }
    // 正面要求: 那条路上【确实】用的是 sink (否命题谁都能满足)
    CHECK(body.find("diagOut()") != std::string::npos);
    CHECK(body.find("diagEmitf(") != std::string::npos);
    PASS();
}

// 【去掉行注释】: 注释里【可以】出现旧措辞 (缺陷说明非把原话抄出来不可, 不然对不上账),
// 被钉死的是【打给人看的字符串】, 不是"谁提过这件事"。所以先剥掉 //...到行尾再断言。
// 带引号感知: 字符串/字符字面量里的 "//" 不算注释头, \" 与 \' 转义也认。
static std::string stripLineComments(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    bool inStr = false, inChr = false;
    for (size_t i = 0; i < s.size(); i++) {
        const char c = s[i];
        if (inStr || inChr) {
            out += c;
            if (c == '\\' && i + 1 < s.size()) { out += s[++i]; continue; }
            if (inStr && c == '"')  inStr = false;
            if (inChr && c == '\'') inChr = false;
            continue;
        }
        if (c == '"')  { inStr = true; out += c; continue; }
        if (c == '\'') { inChr = true; out += c; continue; }
        if (c == '/' && i + 1 < s.size() && s[i + 1] == '/') {
            while (i < s.size() && s[i] != '\n') i++;   // 吃掉注释, 换行留给下一轮
            if (i < s.size()) out += s[i];
            continue;
        }
        out += c;
    }
    return out;
}

// ===== 取"打给人看的文字" (给下面的类级闸门用) =====
// 从(已剥注释的)代码里取出所有【字符串字面量】的内容, 按出现次序首尾相接。屏幕上/文档里出现的
// 就是这些字面量连起来的样子 (相邻的还会被编译器直接粘成一条), 所以判断"这一屏写了什么措辞"
// 扫这一条流就够了 —— 不必解析哪个字面量喂给哪一次打印。转义按编译器那样去掉那个反斜杠。
static std::string printedTextStream(const std::string& code) {
    std::string out;
    for (size_t i = 0; i < code.size(); i++) {
        if (code[i] != '"') continue;
        for (i++; i < code.size(); i++) {
            const char c = code[i];
            if (c == '\\' && i + 1 < code.size()) { out += code[i + 1]; i++; continue; }
            if (c == '"') break;
            out += c;
        }
    }
    return out;
}

// 去掉一个字面量里的箭头 (ASCII 的 -> 与 =>, 以及 U+2192 的 →): 它是指路, 不是比较算子。
// 【为什么必须把箭头剥掉, 而不是拿它当算子】: `->` 里的那个 '>' 会【假冒】比较算子 ——
// `门限 %.4g -> 通过` 若原样交给下面的邻接判据, 命中的是那个无关的 '>' 而不是结论本身。
// 也正因为剥了, 箭头【两侧】的东西会被拉近: 这正是"门限 X -> 通过"这类写法上一版能全绿的
// 原因 (剥掉 '->' 后又没有"通过"这个令牌, 于是无词可用)。所以这里剥箭头与下面加【结论词】
// 是一对, 缺一个就漏。
// 【局限】只认这三种: 别的箭头字形 (⇒ U+21D2 / ➔ / ⟶ …) 与英文措辞一样在盲区里, 见下面
// 那一段"这条闸门到底管到哪"。
static std::string withoutArrows(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); i++) {
        if (s[i] == '-' && i + 1 < s.size() && s[i + 1] == '>') { i++; continue; }
        if (s[i] == '=' && i + 1 < s.size() && s[i + 1] == '>') { i++; continue; }
        // → = UTF-8 E2 86 92 (三个字节一起吃掉, 否则留下半截字节会变成乱码碎片)
        if ((unsigned char)s[i] == 0xE2 && i + 2 < s.size()
            && (unsigned char)s[i + 1] == 0x86 && (unsigned char)s[i + 2] == 0x92) {
            i += 2; continue;
        }
        out += s[i];
    }
    return out;
}

// 【类级判据: 本地不许替库下"统计量与门限"的结论】。
// 命中就返回那一段文字 (给失败信息看), 没命中返回空串。
//   · 门限词: 门限 / 门槛 / 越线 / 超线 / 过线 —— 后三个【自带】那个"线", 只能由"与门限比过"
//     得出, 所以它们一旦出现就命中 (不必旁边还有"门限"二字); "门槛"是"门限"的同义词
//     (2026-09-19 复审补: 同义替换曾是盲区)。
//   · 结论词: 单独一个不算, 必须与门限词【挨着】。两类:
//       (a) 比较算子 < > ≤ ≥ 超过 大于 小于 低于 高于 / 越线 超线 过线;
//       (b) 说结论的措辞 以内 通过 拒绝 √ 未超 未越 —— 2026-09-19 复审补的一组。
//     补 (b) 的理由: 上一版只收算子, 于是"在门限 X 以内" / "门限 X -> 通过" / "门限 X √" /
//     "未超门限 X" 四种【都带结论】的写法全让闸门保持全绿 (实测)。箭头那一类尤其阴: 箭头被
//     withoutArrows 剥掉之后, 连它自带的 '>' 都不在了, 只剩"门限 X 通过"这种无处落脚的形状。
//     "拒绝" 是"通过"的同位词 (判决那一支两边都禁), "未越"是"未超"的同源写法。
//   · 判据: 一个门限词与一个结论词相距不到 W 字节 = 本地替库下了结论。
// 为什么按字节距离而不是"同一个字面量": 一行打印常被拆成两个字面量, 只看单个字面量会漏掉
// 这样写的 `"...= %.4g  <" "  门限 %.4g..."` —— 而这类回归正是闸门要拦的。W 取小: 措辞的
// 邻接是【很近】的 ("超过门限"/"= %.4g 门限"), 拉大只会把无关的两句话配成对。W=10 也因此
// 是这条闸门的量纲: 中间插进别的话就漏 (见下面"这条闸门到底管到哪")。
static std::string localVerdictExcerpt(const std::string& stream) {
    static const char* kLimit[]   = {"门限", "门槛", "越线", "超线", "过线"};
    static const char* kConcl[]   = {"<", ">", "≤", "≥", "超过", "大于", "小于", "低于", "高于",
                                     "越线", "超线", "过线",
                                     "以内", "通过", "拒绝", "√", "未超", "未越"};
    const size_t W = 10;
    for (size_t li = 0; li < sizeof(kLimit) / sizeof(kLimit[0]); li++) {
        const size_t llen = strlen(kLimit[li]);
        for (size_t p = stream.find(kLimit[li]); p != std::string::npos;
             p = stream.find(kLimit[li], p + 1)) {
            const size_t lo = (p > W) ? p - W : 0;      // 窗口: 门限词前后各 W 字节
            const size_t hi = p + llen + W;
            for (size_t ci = 0; ci < sizeof(kConcl) / sizeof(kConcl[0]); ci++) {
                const size_t clen = strlen(kConcl[ci]);
                for (size_t q = stream.find(kConcl[ci]); q != std::string::npos;
                     q = stream.find(kConcl[ci], q + 1)) {
                    if (q < hi && q + clen > lo) {      // 与窗口相交 -> 命中
                        size_t a = (lo > 30) ? lo - 30 : 0;
                        size_t b = (hi + 30 < stream.size()) ? hi + 30 : stream.size();
                        // 别切在多字节字的中间: 失败信息是给人看的, 半截字节只会印成乱码
                        while (a < b && (stream[a] & 0xC0) == 0x80) a++;
                        while (b > a && (stream[b] & 0xC0) == 0x80) b--;
                        return stream.substr(a, b - a);
                    }
                }
            }
        }
    }
    return std::string();
}

// 【同源闸门 (二): 不许替库复述判据 —— 屏幕上那句"拒因"曾经断言了一个它没判过的东西】
// 那一屏末尾的 "→ 拒因: ..." 从前【无条件地】写着 "力通道 χ²/dof = X 超过门限 Y"。而它所在
// 的那个 else 分支, 力通道【完全可以是过了的】: 实机 2026-09-19 18:49:55 那次力 0.6508
// < 门限 2.869 (它过了), 拒绝来自力矩通道 (27.23 > 4.909)。库在同一屏的 stderr 上说对了
// ([Payload] 自检拒绝(模型形式): 力矩通道失拟 ... 超过门限 ...), 而屏幕上那句本地话把操作员
// 支去查一个【没坏】的力通道 —— 同一块里两句话互相打架, 比少一句更坏。
// 判据只有在库里有一份才不会各说各话 (同一函数里 MODEL_FORM_NO_DOF 那个分支自己写下了这条
// 规矩), 所以这里把它做成闸门。
// 【这条闸门的局限, 说在前面】: 它读的是 main.cpp 的【源文本】, 所以钉死的是"那句话怎么写",
// 不是"solveAndApply 跑起来会印出什么" —— 后者要实机数据才走得到, 本测试到不了那一步。
// 下面 (2)(3) 里那几处 find 的串都取得短 (不与换行/缩进较劲), 但改那几行时若把串拆到两个字面量
// 里, 这里会红 —— 那是【故意的】: 它逼改动者回来读一遍这段话。(这些是【词组级】的正面断言;
// 类级的那条规矩在 (1), 它按"文字流上的邻接"判, 拆字面量骗不过它。)
//
// ===== 2026-09-19 加固: 从一个词组升到【一个类】 =====
// 上一版这条闸门只钉死四个字 (`超过门限`), 于是它只看得见【同一个词组】的回归:
//   `χ²/dof = %.4g > 门限 %.4g → 力通道被拒` / `超过 门限` (中间多个空格) / `大于门限` /
//   `越线` —— 四种写法都能骗过它, 而屏幕上照旧误导操作员。
// 更要命的是它【结构上看不见反方向的那个缺陷】: 判决【通过】那一支 (main.cpp 里
// `if (fit.modelFormChecked)`) 从前印的是 "χ²/dof = %.4g  <  门限 %.4g", 一个字都没提
// "超过门限", 却是同一类错 —— 而且错得更危险。力矩那一半在【自由 12 参数模型秩亏】时
// (3n > 12, PayloadCalibration.cpp 明说"力矩通道的失拟检验【没做成】") 从未被检验,
// lackOfFitMomentDof = 0, 于是屏幕上印出 "0  <  0": 一个【假通过】, 还 append 进了
// calib_report.md。假失败把人支去查没坏的通道; 【假通过让人什么都不查】。
// 所以现在扫的是【打给人看的那条文字流】(见上面的 printedTextStream): 一个【门限/门槛词】与一个
// 【结论词】(比较算子, 或"以内/通过/拒绝/√/未超/未越"这类直接说结论的措辞) 挨在一起就红。
// 拆成几个字面量骗不过它 (文字流是首尾相接的, 见上面为什么按字节距离判)。
//
// ===== 【这条闸门到底管到哪】—— 2026-09-19 复审: 这里原来写着"通过/拒绝两支、拆成几个字面量、
//   换成哪个同义词, 都跑不掉"。那句话【是假的】, 已删。复审实测【全绿漏过】的五种写法:
//     `… 在门限 %.4g 以内` / `… 门限 %.4g -> 通过` / `… 门限 %.4g √` / `未超门限 %.4g` / `门槛`
//   —— 其中 `… 门限 %.4g -> 通过` 就是一次【假通过】被写进 calib_report.md 这份永久记录, 而
//   闸门照旧全绿。前四种现已收进令牌表 (箭头两种字形 -> 与 → 都收), "门槛"收进门限词。
//   现在它能抓什么、抓不到什么, 一次说清 (别再让注释比代码能打):
//     · 抓得住: 门限词 ±10 字节内出现令牌表里任何一个【结论词/算子】—— 无论哪一支、无论拆成
//       几个字面量、无论中间夹着 -> 还是 →。这是【中文措辞的令牌匹配 + 邻接】。
//     · 抓不住 (明确的盲区): 令牌表外的措辞 —— 任何【英文写法】("below the limit" / "pass"),
//       以及换了说法又不带门限词的中文 ("合格" / "没超标" / "在允许范围内" / "不存在显著差异");
//       把比较藏在代码里 (先算 bool, 再印 "结论" 二字) —— 本闸门读的是【字面量】, 不是控制流;
//       窗口外的措辞 (门限 X, 详见附表, 以内 —— 中间插进别的话就把邻接撑破);
//       别的箭头字形 (⇒ U+21D2 / ⟶ …) 与别的同义词。
//   ⇒ 它是一道【中文措辞的绊线】, 不是"本地再也不会替库下结论"的证明。改这一屏时人还是要读一遍。
//      (上面 (2)(2b)(3) 那几条正面断言是同一条防线, 各自钉住"该说的话还在不在"。)
static void test_reject_line_does_not_restate_the_library_verdict() {
    TEST(reject_line_does_not_restate_the_library_verdict);
    const std::string src = readWholeFile("../main.cpp");
    CHECK(!src.empty());

    const size_t fn = src.find("static void solveAndApply() {");
    CHECK(fn != std::string::npos);
    const size_t stop = src.find("#if 0  // ===", fn);
    CHECK(stop != std::string::npos);
    const std::string body = src.substr(fn, stop - fn);
    CHECK(body.size() > 15000);

    const std::string code = stripLineComments(body);
    CHECK(code.size() < body.size());          // 剥掉过东西 -> helper 真的在干活

    // (1) 【核心, 类级】打给人看的那条文字流里, 不许有【本地替库下的比较结论】。
    //     (上一版这里是 `code.find("超过门限") == npos` —— 词组级, 看得见那四个字, 看不见
    //      `> 门限` / `大于门限` / `越线`, 也看不见判决通过那一支的 `0 < 0`。)
    const std::string stream = withoutArrows(printedTextStream(code));
    CHECK(stream.size() > 3000);                    // 真的取到了那一屏的字 (取不到先红)
    CHECK(stream.find("门限") != std::string::npos);  // 反面: 门限词确实在流里 (否命题谁都能满足)
    const std::string hit = localVerdictExcerpt(stream);
    if (!hit.empty()) std::printf("\n    本地替库下结论: ...%s...\n", hit.c_str());
    CHECK(hit.empty());

    // (2) 正面要求 (否命题谁都能满足): 还得把人指向权威的那一行, 而且【三种情形】都要有对症的话,
    //     数字也要还在 (钉子只钉"断言", 不钉"报数")。
    CHECK(code.find("以上面 stderr 的 [Payload]") != std::string::npos);  // 指向 stderr 的 [Payload] 行
    CHECK(code.find("力通道被拒") != std::string::npos);                  // 情形一: 力通道过线
    CHECK(code.find("力矩通道失拟被拒") != std::string::npos);            // 情形二: 力矩通道过线
    CHECK(code.find("【没有】模型形式的自检拒绝行") != std::string::npos); // 情形三: 都不是它
    CHECK(code.find("同样见 [Payload] 行") != std::string::npos);
    CHECK(code.find("chi2RepForceRatio") != std::string::npos);           // 两个判据数照旧摆出来
    CHECK(code.find("lackOfFitMomentRatio") != std::string::npos);
    CHECK(code.find("尺子 %.4g N") != std::string::npos);                 // 连同尺子一起

    // (2b) 【判决通过那一支】也要有对症的话 —— 这是正面证据: 少了它, 上面的类级闸门可以靠
    //      "把那一行整个删掉"满足, 而那一屏就再也不报数了。
    CHECK(stream.find("【没有检验】") != std::string::npos);   // dof=0 = 没验过 -> 必须明说
    // 【按"验没验"分两支, 而且两支各按【库自己的那个标志/那个数】分】:
    //   · 通过那一支读库的标志 momentFormChecked —— 库设它就是为了让"没验过"与"验过了"分得开
    //     (PayloadCalibration.cpp:1288-1292)。本地若改用 lackOfFitMomentDof > 0 去【重推】这条
    //     规矩, 就是本提交与 fd4eead 要消灭的那种"本地替库下规矩": 库改了 dof 的约定, 这里会
    //     【不声不响地分错支】。(两者在该分支下等价 —— 库里 modelFormChecked 置 true 之后紧跟
    //     一句 momentFormChecked = (lackOfFitMomentDof > 0)。)
    //   · 拒绝那一支【只能】按 lackOfFitMomentDof > 0 分: 判决被拒时 momentFormChecked 按构造
    //     必为 false (库只在通过那一支置 true), 拿它分会把"失拟真的被算出来且拒了"也读成"没做"。
    CHECK(code.find("fit.momentFormChecked") != std::string::npos);
    CHECK(code.find("fit.lackOfFitMomentDof > 0") != std::string::npos);
    CHECK(code.find("fit.lackOfFitMomentLimit") != std::string::npos);    // 门限照旧摆出来 (报数)
    CHECK(code.find("门限 %.4g") != std::string::npos);                   // 报数的格式还在
    CHECK(stream.find("以 stderr 的 [Payload] 行为准") != std::string::npos);  // 通过那一支也指向库

    // (3) 同一屏上另一个对不上账的数: 姿态级尺子必须【先平方, 再平均, 最后开方】—— repeatSigmaF[]
    //     装的是 σ (N), 不是 σ² (装方差的那个字段叫 repeatSysF[])。从前漏了平方, 于是同一屏上
    //     库打 0.0208 N 而本地打 0.1349 N (sqrt(Σσ/3), 差 6.5 倍), 两个数顶着一个名字。
    CHECK(code.find("fit.repeatSigmaF[0] * fit.repeatSigmaF[0]") != std::string::npos);
    CHECK(code.find("fit.repeatSigmaM[0] * fit.repeatSigmaM[0]") != std::string::npos);
    PASS();
}

// ===== stderr 捕获窗口 =====

static void test_stderr_capture_roundtrip_and_restore() {
    TEST(stderr_capture_roundtrip_and_restore);
    // 窗口【之前】打在 stderr 上的字节, 不该进窗口
    std::fprintf(stderr, "BEFORE-WINDOW (不该被收进窗口)\n");
    std::fflush(stderr);

    const std::string cap1 = tmpPath("cap1.tmp");
    std::string out;
    CHECK(SessionReport::stderrCaptureBegin(cap1.c_str()));
    CHECK(SessionReport::stderrCaptureActive());
    std::fprintf(stderr, "  [Payload] 逐姿态残差 (力 N / 力矩 N·m):\n");
    std::fprintf(stderr, "         * pose  3:   0.0224    0.0013   1.83\n");
    std::fflush(stderr);
    CHECK(SessionReport::stderrCaptureEnd(&out));
    CHECK(!SessionReport::stderrCaptureActive());
    CHECK(out == "  [Payload] 逐姿态残差 (力 N / 力矩 N·m):\n"
                 "         * pose  3:   0.0224    0.0013   1.83\n");
    CHECK(out.find("BEFORE-WINDOW") == std::string::npos);
    CHECK(_access(cap1.c_str(), 0) != 0);          // 临时文件收尾时删掉了

    // 【还原了没有】: 再开一个窗口。若 fd 2 还指着第一个临时文件, 这里就会把"上一次的字节"
    // 再收一遍 —— 那正是"诊断被弄丢/串台"的样子。
    std::string out2;
    const std::string cap2 = tmpPath("cap2.tmp");
    CHECK(SessionReport::stderrCaptureBegin(cap2.c_str()));
    CHECK(SessionReport::stderrCaptureEnd(&out2));
    CHECK(out2.empty());
    CHECK(_access(cap2.c_str(), 0) != 0);

    // 窗口【之外】的 stderr 仍然通 (这一行只会出现在控制台上)
    std::fprintf(stderr, "AFTER-WINDOW (应当只出现在控制台上, 不进任何窗口)\n");
    std::fflush(stderr);

    // out == nullptr 也要能收尾 (调用方只想要还原的场合)
    CHECK(SessionReport::stderrCaptureBegin(tmpPath("cap3.tmp").c_str()));
    std::fprintf(stderr, "丢弃\n");
    std::fflush(stderr);
    CHECK(SessionReport::stderrCaptureEnd(nullptr));
    CHECK(!SessionReport::stderrCaptureActive());
    PASS();
}

// Begin 失败时【stderr 一个字节都不动】: 那是"没搭起窗口", 不是"把 stderr 吞了"
static void test_stderr_capture_begin_failure_leaves_stderr_alone() {
    TEST(stderr_capture_begin_failure_leaves_stderr_alone);
    CHECK(!SessionReport::stderrCaptureBegin(nullptr));
    CHECK(!SessionReport::stderrCaptureBegin(""));
    CHECK(!SessionReport::stderrCaptureBegin("no_such_dir\\nope.tmp"));
    CHECK(!SessionReport::stderrCaptureActive());

    std::fprintf(stderr, "STILL-ALIVE (窗口没搭起来, 这一行照旧出去)\n");
    std::fflush(stderr);

    // 紧接着开一个【能搭起来】的窗口: 上面那一行不该出现在它里面 (说明 stderr 没被吞)
    std::string out;
    CHECK(SessionReport::stderrCaptureBegin(tmpPath("cap4.tmp").c_str()));
    std::fprintf(stderr, "窗口里的\n");
    std::fflush(stderr);
    CHECK(SessionReport::stderrCaptureEnd(&out));
    CHECK(out == "窗口里的\n");

    // 套娃要被拒绝: 窗口里再 Begin 返回 false, 且不破坏外层窗口
    CHECK(SessionReport::stderrCaptureBegin(tmpPath("cap5.tmp").c_str()));
    CHECK(!SessionReport::stderrCaptureBegin(tmpPath("cap6.tmp").c_str()));
    std::string out2;
    CHECK(SessionReport::stderrCaptureEnd(&out2));
    CHECK(out2.empty());
    PASS();
}

// 【收不回来时【不删】那个临时文件】—— 字节不能被销毁, 只能被"没记上"
// 危险的那一条路: Begin 成功 (fd 2 已指向临时文件) 之后, End 里那一句 fflush(stderr) 会把
// stderr 缓冲里的字节【推进文件】, 然后 _open 读那一步失败。此刻盘上那份是这些字节【唯一的
// 副本】(它们已经没进控制台了, 因为窗口里 fd 2 不指向控制台)。从前这里照样 _unlink —— 于是
// 控制台与文档块两头都没有, 只留一句"收回来了但读不出来"。
// 这里把读那一步弄失败: 给一个【相对】路径, Begin 之后换掉工作目录, 那一句 _open 就找不到它了。
// (这是真的在跑那条失败路径, 不是在旁边断言一句"理论上会留": 断言里连文件内容都比了。)
static void test_stderr_capture_end_read_failure_keeps_the_bytes() {
    TEST(stderr_capture_end_read_failure_keeps_the_bytes);
    const std::string rel = tmpPath("cap_fail.tmp");    // 相对路径: 靠改工作目录把它弄失效
    _unlink(rel.c_str());

    CHECK(SessionReport::stderrCaptureBegin(rel.c_str()));
    std::fprintf(stderr, "END-READ-FAIL 这一段收不回来, 但必须留在盘上\n");
    std::fflush(stderr);

    char cwd[1024];
    CHECK(_getcwd(cwd, sizeof(cwd)) != nullptr);
    CHECK(_chdir(TMPDIR) == 0);                         // 于是 rel 这个相对路径解析到别处去了

    std::string out;
    const bool ok = SessionReport::stderrCaptureEnd(&out);
    CHECK(_chdir(cwd) == 0);                            // 先换回来 (后面哪个断言红了也不留在里面)

    CHECK(!ok);                                         // 读不出来 -> 照实返回 false
    CHECK(out.empty());
    CHECK(!SessionReport::stderrCaptureActive());       // 还原是无条件的

    const std::string leftover = SessionReport::stderrCaptureLeftoverPath();
    // 路径报得出来 (调用方要写进块尾), 而且【报的就是盘上那个文件】—— 它不再是基名本身, 而是基名
    // 的一个本次专属变体 (F2): 那个固定名字会被下一次 Begin 截掉, 所以名字必须每次都不一样 (见
    // test_stderr_capture_leftover_survives_a_second_begin)。基名还在里面, 看得出是谁留下的。
    const std::string stem = rel.substr(0, rel.size() - 4);   // 去掉 ".tmp"
    CHECK(leftover.compare(0, stem.size(), stem) == 0);
    CHECK(leftover.size() > rel.size());                // ≠ 那个固定名字
    CHECK(_access(rel.c_str(), 0) != 0);                // 基名【从来没被建过】—— 不留残骸
    CHECK(_access(leftover.c_str(), 0) == 0);           // 【文件还在】—— 没被删掉
    // 【字节一个不少】: 这一段此刻只有这一个副本, 读回来就是"可捡回"的证明
    CHECK(readWholeFile(leftover) == "END-READ-FAIL 这一段收不回来, 但必须留在盘上\n");

    // 【还原了没有】: 再开一个窗口, 收到的只能是新字节 (串台 = 上一次的字节又收一遍)
    std::string out2;
    CHECK(SessionReport::stderrCaptureBegin(tmpPath("cap7.tmp").c_str()));
    std::fprintf(stderr, "新的窗口\n");
    std::fflush(stderr);
    CHECK(SessionReport::stderrCaptureEnd(&out2));
    CHECK(out2 == "新的窗口\n");
    CHECK(SessionReport::stderrCaptureLeftoverPath()[0] == '\0');  // 成功的那次不留东西

    _unlink(leftover.c_str());                          // 收尾 (它本该留在盘上; 测试自己清)
    PASS();
}

// 【读到一半失败 ≠ EOF】(N4) —— 第二条"收不回来"的路, 与上一条(_open 失败)不同:
// 上一条连文件都没打开, 这一条是【已经读进来半截了才坏掉】(I/O 错误 / 杀软正占着这个文件)。
// 旧代码里两者长得一模一样: `while ((n = _read(...)) > 0) {...}` 之后无条件 _close + _unlink ——
// 读坏了与读完了分不出来: out 里是半截、ok 却是 true, 调用方把那半截当成整段发出去 (没有警告),
// 临时文件还被删掉, 缺的尾巴【安静地没了】。
// 修法: n < 0 与 _open 失败【同等对待】—— ok = false / 文件不删 / 路径报出去 / 半截一个字都不并入。
// 这条用例【真的在读的那一步注错】(见本文件顶上的注错点): 先给 4 个字节 (制造"半截"), 再返回 -1。
static void test_stderr_capture_end_read_error_is_a_failure_not_eof() {
    TEST(stderr_capture_end_read_error_is_a_failure_not_eof);
    const std::string path = tmpPath("cap_read_err.tmp");
    _unlink(path.c_str());

    CHECK(SessionReport::stderrCaptureBegin(path.c_str()));
    const std::string line = "READ-ERROR: 这一段读到一半坏了 —— 半截不许当成整段发出去\n";
    std::fprintf(stderr, "%s", line.c_str());
    std::fflush(stderr);

    // 从这一句起, 头文件里那一句 _read 先给 4 个字节, 再一律返回 -1 (= 中途 I/O 出错)
    const int  callsBefore = g_fakeReadCalls;
    const int  errsBefore  = g_fakeReadErrors;
    const long bytesBefore = g_fakeBytesServed;
    g_partialFirstRead = 4;
    g_failRead = true;
    std::string out;
    const bool ok = SessionReport::stderrCaptureEnd(&out);
    g_failRead = false;                     // 无论断言怎么走, 都不给后面的用例留状态
    g_partialFirstRead = 0;

    // 【先证明这条路径真的被走到了】: 头文件那一句 _read 确实过了注错点 ——
    // 先交出去 4 个字节 (半截真的进过 out), 再返回 -1 (读坏了)。
    CHECK(g_fakeReadCalls > callsBefore);
    CHECK(g_fakeBytesServed == bytesBefore + 4);
    CHECK(g_fakeReadErrors == errsBefore + 1);

    CHECK(!ok);                                   // 读坏了 -> 照实返回 false (旧代码这里是 true)
    CHECK(out.empty());                           // ← 半截【不许】出去 (旧代码这里是那 4 个字节)
    CHECK(!SessionReport::stderrCaptureActive()); // 还原是无条件的

    const std::string leftover = SessionReport::stderrCaptureLeftoverPath();
    const std::string stem = path.substr(0, path.size() - 4);   // 去掉 ".tmp"
    CHECK(leftover.compare(0, stem.size(), stem) == 0);         // 路径报得出来 (调用方写进块尾)
    CHECK(leftover.size() > path.size());                       // 本次专属变体, 不是那个固定名字
    CHECK(_access(path.c_str(), 0) != 0);                       // 基名【从来没被建过】
    CHECK(_access(leftover.c_str(), 0) == 0);     // 【文件还在】—— 旧代码把它 _unlink 掉了
    CHECK(readWholeFile(leftover) == line);       // 一个字节不少 (半截那 4 个也在里面)

    _unlink(leftover.c_str());                    // 收尾 (它本该留在盘上; 测试自己清)
    PASS();
}

// ===== F2: 一次捕获的残留【必须扛得住下一次 Begin】=====
//
// 失败那一路的临时文件是那些字节的【唯一副本】: 窗口里 fd 2 指着它, 所以它们【没进控制台】;
// 文档块里只落了一句"收不回来"。而它的路径刚被写进块尾交给操作员。此时操作员最可能做的下一件
// 事就是【再按一次 's' 看一遍】—— 从前那个名字是【固定的】(calib_stderr.tmp)、Begin 又是
// _O_CREAT|_O_TRUNC, 于是那一次按键【当场把刚刚指着的那份副本截成 0 字节】: 两头都没有了。
// 这个用例把"同一个基名再开一次, 残留一个字节不少"钉死 (名字每次不同 + _O_EXCL 不碰已存在文件)。
static void test_stderr_capture_leftover_survives_a_second_begin() {
    TEST(stderr_capture_leftover_survives_a_second_begin);
    const std::string base = tmpPath("cap_leftover.tmp");
    _unlink(base.c_str());

    // --- 第一次捕获: 用"换工作目录"把 End 里那一句 _open 弄失效, 留下残留 ---
    CHECK(SessionReport::stderrCaptureBegin(base.c_str()));
    const std::string keep = "LEFT-OVER-1 这是唯一副本, 下一次 Begin 一个字节也不许动它\n";
    std::fprintf(stderr, "%s", keep.c_str());
    std::fflush(stderr);

    char cwd[1024];
    CHECK(_getcwd(cwd, sizeof(cwd)) != nullptr);
    CHECK(_chdir(TMPDIR) == 0);
    std::string out1;
    CHECK(!SessionReport::stderrCaptureEnd(&out1));
    CHECK(_chdir(cwd) == 0);
    CHECK(out1.empty());

    const std::string leftover1 = SessionReport::stderrCaptureLeftoverPath();
    CHECK(!leftover1.empty());
    CHECK(_access(leftover1.c_str(), 0) == 0);
    CHECK(readWholeFile(leftover1) == keep);        // 残留此刻是完好的
    CHECK(_access(base.c_str(), 0) != 0);           // 而且用的不是那个固定名字

    // --- 第二次捕获: 【同一个基名】再开一次 (= 操作员"再按一次 's'") ---
    CHECK(SessionReport::stderrCaptureBegin(base.c_str()));
    const std::string second = "窗口二的内容\n";
    std::fprintf(stderr, "%s", second.c_str());
    std::fflush(stderr);
    std::string out2;
    CHECK(SessionReport::stderrCaptureEnd(&out2));
    CHECK(out2 == second);                          // 没串台: 只收到第二次的字节

    // --- 残留【一个字节没少】: 旧的 _O_CREAT|_O_TRUNC + 固定名字在这里会把它截成 0 ---
    CHECK(_access(leftover1.c_str(), 0) == 0);
    CHECK(readWholeFile(leftover1) == keep);
    CHECK(SessionReport::stderrCaptureLeftoverPath()[0] == '\0');   // 成功的那次不留东西

    _unlink(leftover1.c_str());                     // 收尾 (它本该留在盘上; 测试自己清)
    _unlink(base.c_str());
    PASS();
}

// 命名规则本身 (纯函数): 每次捕获一个专属名字, 基名与扩展名都还认得出
static void test_capture_tmp_path_is_unique_and_recognizable() {
    TEST(capture_tmp_path_is_unique_and_recognizable);
    const std::string a = SessionReport::captureTmpPathFor("calib\\calib_stderr.tmp", 4188, 1);
    const std::string b = SessionReport::captureTmpPathFor("calib\\calib_stderr.tmp", 4188, 2);
    CHECK(a != b);                                             // 序号不同 -> 名字不同
    const std::string pfx = "calib\\calib_stderr.";
    CHECK(a.compare(0, pfx.size(), pfx) == 0);                 // 基名还在 (看得出是谁留下的)
    CHECK(a.find("4188_1") != std::string::npos);              // 进程号 + 序号
    CHECK(a.compare(a.size() - 4, 4, ".tmp") == 0);            // 扩展名还认得出
    CHECK(a.size() > strlen("calib\\calib_stderr.tmp"));       // ≠ 那个固定名字

    // 没有扩展名 -> 接在末尾; 目录名里的 '.'【不算】扩展名 (别插到目录上去)
    CHECK(SessionReport::captureTmpPathFor("no_ext_name", 7, 3) == "no_ext_name.7_3");
    CHECK(SessionReport::captureTmpPathFor("D:\\a.b\\cap", 7, 3) == "D:\\a.b\\cap.7_3");
    PASS();
}

int main() {
    std::printf("=== SessionReport Tests ===\n");
    _mkdir(TMPDIR);

    test_block_header_format();
    test_block_wraps_body_verbatim();
    test_block_closes_fence_when_body_has_no_trailing_newline();
    test_block_trailer_goes_after_the_fence();
    test_block_wraps_a_real_console_screen_verbatim();
    test_payload_d_section_prints_both_sign_conventions();
    test_payload_d_section_qualifies_a_rejected_fit();
    test_payload_d_section_negative_cs_prints_no_fabricated_signs();
    test_append_preserves_existing_bytes();
    test_append_creates_missing_file();
    test_append_on_unreadable_file_reports_failure_without_truncating();
    test_solve_path_prints_only_through_the_sink();
    test_reject_line_does_not_restate_the_library_verdict();
    test_stderr_capture_roundtrip_and_restore();
    test_stderr_capture_begin_failure_leaves_stderr_alone();
    test_stderr_capture_end_read_failure_keeps_the_bytes();
    test_stderr_capture_end_read_error_is_a_failure_not_eof();
    test_stderr_capture_leftover_survives_a_second_begin();
    test_capture_tmp_path_is_unique_and_recognizable();

    // 清理 (文件先删, 目录才删得掉)
    _unlink(tmpPath("append.md").c_str());
    _unlink(tmpPath("fresh.md").c_str());
    _unlink(tmpPath("readonly.md").c_str());
    _rmdir(TMPDIR);

    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed ? 1 : 0;
}
