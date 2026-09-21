#pragma once
// ===== 30004 实时帧的【帧级自检】—— 唯一一份定义 =====
//
// 【为什么要有它 (2026-09-21)】
//   30004 是【字节流】: TCP 不保证一次 recv 给一整帧。读取层用的是一条"宁断不错"的约定
//   (见 RobotConnection.cpp 的 robotRecvRealtime): 只有【恰好一整帧】才收下, 其余一律断开
//   重连 —— 那条路确实保证不会错位 (重连后从新流的开头读)。审计见
//   specs/2026-09-21-raw-channel-calibration-run-005.md §12。
//
//   ⚠ 但"恰好 1440 字节"【不能证明】那段字节落在帧边界上。若机械臂侧的布局与我们的假设不符
//     (固件改了 / 字段增减), 我们会【照旧读满 1440】, 然后按错位的偏移解析 —— 而没有任何
//     东西会报警。整个链路上唯一的守卫是 RelayCore 里那个 `sane` (只验 @1168/@1176 四个负载
//     字段), 而【力数据 @576/@720/@1304 写在它【外面】】⇒ 错位帧会污染力读数。
//
//   ⇒ 厂商文档早就给了用来做这件事的字段 (TCP_IP远程控制接口文档.md:826):
//        TestValue | uint64 | 0048 ~ 0055 | 0x0123456789ABCDEF
//     2026-09-21 之前, 这个魔数【全库一次都没被用过】。本文件把它用起来。
//
// 【它做什么】把"这一帧看起来是有效的 30004 帧吗"收成【一个纯谓词】, 于是
//   · 读取层可以在一帧数据写入任何字段【之前】判掉它;
//   · 它可以被单元测试直接调用 (tests/test_frame_layout.cpp), 不需要 socket。
//
// 【它不做什么 —— 如实说】它【不】逐字段校验各偏移; 魔数只能锚住 offset 48。偏移的正确性
//   另有【闭环】证据: 我们下发的负载参数被原样回读 (@1168 Load = 0.404 kg、
//   @1176 CenterX/Y/Z = (0.3, -0.1, 68.7) mm —— 三个任意小数逐位相等)。那条比"某个魔数
//   匹配上了"强得多, 出处同上 §12.3。
//
// 【字节序: 为什么要接受两种】
//   文档给的是数值 0x0123456789ABCDEF, 【没说】它在线路上怎么排。x86 惯例是小端 (同一条
//   流里的 double 也是小端, 否则本客户端读不出任何合理值), 所以小端是【很可能】的 ——
//   但"很可能"不是"已验证"。而猜错的代价是严重的: 若只认一种而猜反, 每一帧都会被判坏。
//   ⇒ 两种都接受, 并把【实际看到的是哪一种】报出来 (classifyMagic 的返回值给了调用方)。
//     于是"机器人怎么序列化 uint64"这个问题由【数据】回答, 而不是由我们的假设回答 ——
//     这正是本项目反复吃过账的那一类("前提要回源头核")。
//
// 【档位: 默认"只报不判"】
//   与"闸门表先并排报 comp−@720、看清参考量之前不改判据"是【同一套做法】: 在一个量的
//   真值还没被验证之前, 先只观察、不拿它做判决。理由同 byte-order 那一段 —— 我们还没在
//   真机上见过这个字段。档位取值见 Config::FORCE_FRAME_MAGIC_MODE。

#include <cstdint>

namespace FrameLayout {

    // 30004 帧长。出处: TCP_IP远程控制接口文档.md:63
    //   "客户端每 8ms 接收一次机器人实时状态信息 (1440字节)"
    // 与 Config::FORCE_EFFECTIVE_SAMPLE_RATE = 125 (Hz) 互为倒数 —— 两者一致。
    const int LEN_30004 = 1440;

    // 帧内自检字段 TestValue 的位置与值。出处: 同文档 :826 (字节区间 0048 ~ 0055)。
    const int      MAGIC_OFFSET = 48;
    const uint64_t MAGIC_VALUE  = 0x0123456789ABCDEFULL;

    // 帧里那个 uint64 的字节序 —— 由实读结果反推, 不是假定。
    enum class MagicOrder { None = 0, LittleEndian = 1, BigEndian = 2 };

    // 按【小端】把 8 个字节拼成 uint64。
    // ⚠ 不写成 reinterpret_cast<uint64_t*>(p) 再解引用, 两个理由:
    //   · 不依赖 p 的对齐 (标准上那不是任何东西保证的; RobotConnection 的 buf 是 char 数组);
    //   · 意图写在字面上 —— 这里要的是"照小端的读法还原", 不是一个跟平台相关的行为。
    inline uint64_t readU64LE(const unsigned char* p) {
        uint64_t v = 0;
        for (int i = 7; i >= 0; --i) v = (v << 8) | (uint64_t)p[i];
        return v;
    }

    // 判 offset 48 那个 uint64 是哪种排法。契约:
    //   len != LEN_30004  -> None  (短读/半帧; 【不再往下读】, 所以不会越界)
    //   小端读到 MAGIC    -> LittleEndian
    //   大端读到 MAGIC    -> BigEndian
    //   都不是            -> None
    inline MagicOrder classifyMagic(const unsigned char* buf, int len) {
        if (buf == nullptr)        return MagicOrder::None;
        if (len != LEN_30004)      return MagicOrder::None;   // 先判长度: 保证下面不越界
        if (readU64LE(buf + MAGIC_OFFSET) == MAGIC_VALUE) return MagicOrder::LittleEndian;
        uint64_t be = 0;
        for (int i = 0; i < 8; ++i) be = (be << 8) | (uint64_t)buf[MAGIC_OFFSET + i];
        if (be == MAGIC_VALUE)     return MagicOrder::BigEndian;
        return MagicOrder::None;
    }

    // "这一帧是有效的 30004 帧吗"。= classifyMagic 不为 None。
    // 调用方拿到 true 就可以当"这是一整帧、而且认得它的布局"用。
    inline bool frameLooksValid30004(const unsigned char* buf, int len) {
        return classifyMagic(buf, len) != MagicOrder::None;
    }

}
