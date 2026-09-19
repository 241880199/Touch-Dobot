#pragma once
#include <cstddef>

// 标定文件放哪。
//
// 为什么需要它: 三个标定文件原本都用相对路径 ("./payload_calib.json") 读写,
// 落在【当前工作目录】—— 从 Touch_Client\ 启动和从 x64\Release\ 启动会拿到
// 两份不同的文件。
//
// 注: 本模块【不管有效期】。标定是否仍然可信由启动自检实测判定 (见 main.cpp),
//     不用时间闸门 —— 时间只是代理指标, 而这里能直接测。
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
    const char* fileFor(const char* name);
}
