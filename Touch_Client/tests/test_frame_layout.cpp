// Standalone test: FrameLayout — 30004 帧自检 (TestValue @48) 的纯谓词
// Build: build_frame_layout_test.bat
// Run: test_frame_layout.exe
//
// 【为什么这些用例值得写】这不是"给一行比较加个测试"。谓词的两个输入维度 (长度 / 魔数)
// 各自都有【判错的后果】, 而后果不对称:
//   · 判成 false 而其实是好帧 -> 帧被丢掉 (档 2 下还会重连) -> 现场数据断了;
//   · 判成 true 而其实是错位帧 -> 力读数被污染, 而且【没有任何别的东西会报警】。
// 所以两个方向都要有用例, 边界 (长度不足时不越界读) 单列。
//
// ⚠ 本文件【不碰 socket】: FrameLayout.h 是纯头文件, 这正是"把判定抽成纯函数"的目的
//   (与 force/ZeroDriftCheck.h 同一套做法)。

#include <iostream>
#include <cstring>

#include "../robot/FrameLayout.h"

static int g_passed = 0, g_failed = 0;

#define TEST(name) do { std::cout << "  " << #name << "... "; } while(0)
#define PASS() do { std::cout << "PASS" << std::endl; g_passed++; } while(0)
#define CHECK(cond) do { if (!(cond)) { std::cout << "FAIL: " << #cond << std::endl; g_failed++; return; } } while(0)

using namespace FrameLayout;

// 造一个 1440 字节的帧。order: 0 = 什么都不写, 1 = 小端写魔数, 2 = 大端写魔数。
// 其余字节填一个【非魔数】的花纹 —— 免得"全 0 恰好被读成魔数"这类巧合混进来。
static void makeFrame(unsigned char* buf, int len, int order) {
    for (int i = 0; i < len; i++) buf[i] = (unsigned char)(0xA5 ^ (i & 0xFF));
    if (order == 0 || len < MAGIC_OFFSET + 8) return;
    uint64_t v = MAGIC_VALUE;
    for (int i = 0; i < 8; i++) {
        if (order == 1) buf[MAGIC_OFFSET + i] = (unsigned char)((v >> (8 * i)) & 0xFF);
        else             buf[MAGIC_OFFSET + i] = (unsigned char)((v >> (8 * (7 - i))) & 0xFF);
    }
}

// ===== 正面: 两种字节序都要认 =====
static void test_little_endian_ok() {
    TEST(little_endian_ok);
    unsigned char buf[LEN_30004];
    makeFrame(buf, LEN_30004, 1);
    CHECK(classifyMagic(buf, LEN_30004) == MagicOrder::LittleEndian);
    CHECK(frameLooksValid30004(buf, LEN_30004));
    PASS();
}

static void test_big_endian_ok() {
    TEST(big_endian_ok);
    unsigned char buf[LEN_30004];
    makeFrame(buf, LEN_30004, 2);
    // 为什么要认大端: 文档只给了【数值】, 没说线路上怎么排。只认一种而猜反, 每一帧都会被
    // 判坏 (档 2 下就是永久重连)。见 FrameLayout.h 顶上那一段。
    CHECK(classifyMagic(buf, LEN_30004) == MagicOrder::BigEndian);
    CHECK(frameLooksValid30004(buf, LEN_30004));
    PASS();
}

// ===== 反面: 魔数不对 =====
static void test_no_magic() {
    TEST(no_magic);
    unsigned char buf[LEN_30004];
    makeFrame(buf, LEN_30004, 0);
    CHECK(classifyMagic(buf, LEN_30004) == MagicOrder::None);
    CHECK(!frameLooksValid30004(buf, LEN_30004));
    PASS();
}

static void test_off_by_one_byte() {
    TEST(off_by_one_byte);
    unsigned char buf[LEN_30004];
    makeFrame(buf, LEN_30004, 1);
    buf[MAGIC_OFFSET] ^= 0x01;              // 只翻最低位的一个 bit
    CHECK(classifyMagic(buf, LEN_30004) == MagicOrder::None);
    PASS();
}

static void test_magic_shifted_by_one() {
    TEST(magic_shifted_by_one);
    // "整段错了一个字节"——正是我们真正怕的那种失效。它必须被判出来。
    unsigned char buf[LEN_30004];
    makeFrame(buf, LEN_30004, 1);
    unsigned char shifted[LEN_30004];
    memset(shifted, 0, sizeof(shifted));
    memcpy(shifted + 1, buf, LEN_30004 - 1);   // 整体右移一个字节
    CHECK(classifyMagic(shifted, LEN_30004) == MagicOrder::None);
    PASS();
}

// ===== 边界: 长度 =====
static void test_len_short_of_magic() {
    TEST(len_short_of_magic);
    unsigned char buf[128];
    makeFrame(buf, sizeof(buf), 1);
    // 55 < 48+8 ⇒ 若实现忘了先判长度, 这里就是一次越界读。必须返回 None。
    CHECK(classifyMagic(buf, MAGIC_OFFSET + 7) == MagicOrder::None);
    CHECK(!frameLooksValid30004(buf, MAGIC_OFFSET + 7));
    PASS();
}

static void test_len_one_short() {
    TEST(len_one_short);
    unsigned char buf[LEN_30004];
    makeFrame(buf, LEN_30004, 1);
    // 1439 —— 正是"半帧/短读"那一档。必须拒 (否则会拿一段不完整的字节当一整帧用)。
    CHECK(classifyMagic(buf, LEN_30004 - 1) == MagicOrder::None);
    PASS();
}

static void test_len_one_long() {
    TEST(len_one_long);
    unsigned char buf[LEN_30004 + 1];
    makeFrame(buf, sizeof(buf), 1);
    // 1441 —— 【也必须拒】: 契约是"正好一整帧"。多出来的那一字节说明我们对流的分段理解
    // 与事实不符, 那正是要早知道的事。
    CHECK(classifyMagic(buf, LEN_30004 + 1) == MagicOrder::None);
    PASS();
}

static void test_null_buffer() {
    TEST(null_buffer);
    CHECK(classifyMagic(nullptr, LEN_30004) == MagicOrder::None);
    CHECK(!frameLooksValid30004(nullptr, LEN_30004));
    PASS();
}

// ===== 读法本身 =====
static void test_readU64LE() {
    TEST(readU64LE);
    const unsigned char le[8] = {0xEF, 0xCD, 0xAB, 0x89, 0x67, 0x45, 0x23, 0x01};
    CHECK(readU64LE(le) == 0x0123456789ABCDEFULL);
    const unsigned char allZero[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    CHECK(readU64LE(allZero) == 0ULL);
    const unsigned char allFF[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    CHECK(readU64LE(allFF) == 0xFFFFFFFFFFFFFFFFULL);
    PASS();
}

static void test_all_ff_frame_rejected() {
    TEST(all_ff_frame_rejected);
    // 全 0xFF 的帧是最容易被"放宽一点"的实现误收的输入之一。
    unsigned char buf[LEN_30004];
    memset(buf, 0xFF, sizeof(buf));
    CHECK(classifyMagic(buf, LEN_30004) == MagicOrder::None);
    PASS();
}

static void test_all_zero_frame_rejected() {
    TEST(all_zero_frame_rejected);
    unsigned char buf[LEN_30004];
    memset(buf, 0x00, sizeof(buf));
    CHECK(classifyMagic(buf, LEN_30004) == MagicOrder::None);
    PASS();
}

int main() {
    std::cout << "--- FrameLayout (30004 帧自检) ---" << std::endl;
    test_readU64LE();
    test_little_endian_ok();
    test_big_endian_ok();
    test_no_magic();
    test_off_by_one_byte();
    test_magic_shifted_by_one();
    test_len_short_of_magic();
    test_len_one_short();
    test_len_one_long();
    test_null_buffer();
    test_all_ff_frame_rejected();
    test_all_zero_frame_rejected();
    std::cout << "\nResults: " << g_passed << " passed, " << g_failed << " failed" << std::endl;
    return g_failed == 0 ? 0 : 1;
}
