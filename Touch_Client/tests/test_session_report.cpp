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

#include "../core/SessionReport.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <direct.h>   // _mkdir
#include <io.h>       // _access / _unlink / _rmdir
#include <sys/stat.h> // _chmod / _S_IREAD / _S_IWRITE

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
// 从前这里只打一支, 而且块里的句子说"若实际反向, d 会整体变负" —— 那是算错的 (反向给的是另一个
// 正数), 会把操作员引到错的地方。这个用例把"两支都打 + 不给单一结论"钉住。
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

    // 【那句算错的话不许再出现】: 反向约定给的是另一个正数, 不是负数
    CHECK(s.find("变负") == std::string::npos);
    snprintf(buf, sizeof(buf), "%.17g", czRobot + csZ);
    CHECK(czRobot + csZ > 0.0);            // 事实层面: 反向那一支是正的 (run-001: 124.256 mm)
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
    CHECK(leftover == rel);                             // 路径报得出来 (调用方要写进块尾)
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
    test_append_preserves_existing_bytes();
    test_append_creates_missing_file();
    test_append_on_unreadable_file_reports_failure_without_truncating();
    test_solve_path_prints_only_through_the_sink();
    test_stderr_capture_roundtrip_and_restore();
    test_stderr_capture_begin_failure_leaves_stderr_alone();
    test_stderr_capture_end_read_failure_keeps_the_bytes();

    // 清理 (文件先删, 目录才删得掉)
    _unlink(tmpPath("append.md").c_str());
    _unlink(tmpPath("fresh.md").c_str());
    _unlink(tmpPath("readonly.md").c_str());
    _rmdir(TMPDIR);

    std::printf("\n%d passed, %d failed\n", g_passed, g_failed);
    return g_failed ? 1 : 0;
}
