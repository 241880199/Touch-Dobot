// glut.h 修复: 新版 Windows SDK (10.0.26100+) 的 stdlib.h 用 _Noreturn 声明 exit，
// 与 OpenHaptics SDK 自带的 GLUT 3.2 中 __declspec(noreturn) 声明冲突。
// 此文件必须在所有 GL/glut.h 之前被包含。

#pragma once

// 阻止 glut.h 的 atexit hack (避免 glutInit 等函数被宏替换)
#define GLUT_DISABLE_ATEXIT_HACK

// 强制 _CRTIMP 为空，避免 exit 声明带 __declspec(dllimport)
// 从而与 UCRT 的 exit 声明兼容
#ifdef _CRTIMP
#undef _CRTIMP
#endif
#define _CRTIMP

#include <GL/glut.h>
