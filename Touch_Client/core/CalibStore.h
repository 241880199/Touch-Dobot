#pragma once
#include <cstddef>

// 标定文件的生命周期: 放哪、还能不能用。
//
// 为什么需要它: 三个标定文件原本都用相对路径 ("./payload_calib.json") 读写,
// 落在【当前工作目录】—— 从 Touch_Client\ 启动和从 x64\Release\ 启动会拿到
// 两份不同的文件。而且标定结果永不过期, 一份几天前的数据会被当成权威值。
// 路径解析和有效期判定放在同一个模块, 因为它们回答的是同一个问题:
// "该不该用这份标定"。
namespace CalibStore {

    // 纯函数: 由可执行文件全路径推出标定目录 (结尾带反斜杠)。
    // 规则: 去掉文件名 + 上溯两级 (上溯会停在盘符根, 不会再往上剥), 再拼 "calib\"。
    //   ...\Touch_Client\x64\Release\Touch_Client.exe → ...\Touch_Client\calib\  (结尾带反斜杠)
    //   C:\a\b.exe                                    → C:\a\calib\   (只上溯得了一级)
    // 一个反斜杠都没有 (无法确定目录) 时返回 false (out 内容未定义)。
    bool deriveDir(const char* exePath, char* out, size_t outSize);

    // 标定目录绝对路径, 结尾带反斜杠。不存在时创建。进程内缓存。
    const char* dir();

    // 拼接标定文件绝对路径。静态缓冲, 下次调用即失效。
    // 只拼路径, 不做任何新鲜度判定 —— 写入用这个。
    const char* fileFor(const char* name);

    // 解析一个标定文件是否可用。
    //   可用   → 返回绝对路径 (静态缓冲, 下次调用即失效)
    //   过期   → 改名为 <name>.expired, 打印醒目提示, 返回 nullptr
    //   不存在 → 返回 nullptr (静默; 首次启动没有标定文件是正常情况)
    // 判据见 isFresh: 没有 saved_at_unix 字段一律视为过期。
    const char* resolve(const char* name);

    // 纯函数, 便于单测。nowUnix 由调用方传入, 内部不读时钟。
    // savedAtUnix <= 0 (缺字段/解析失败) 判过期; 时钟回拨按 0 年龄处理。
    bool isFresh(long savedAtUnix, long nowUnix, long maxAgeSec);
}
