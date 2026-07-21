#pragma once

// UI 布局常量
namespace HudLayout {
    constexpr int TOP_BAR_H = 30;
    constexpr int PANEL_Y = TOP_BAR_H + 2;
    constexpr int PANEL_H = 768 - PANEL_Y;

    // 左栏: 指令 + 反馈
    constexpr int LEFT_X = 4;
    constexpr int LEFT_W = 340;

    // 中栏: 力数据
    constexpr int CENTER_X = LEFT_X + LEFT_W + 4;
    constexpr int CENTER_W = 340;

    // 右栏: 3D + 坐标
    constexpr int RIGHT_X = CENTER_X + CENTER_W + 4;
    constexpr int RIGHT_W = 1024 - RIGHT_X - 4;
    constexpr int RIGHT_3D_H = 470;
    constexpr int RIGHT_BOTTOM_Y = PANEL_Y + RIGHT_3D_H + 4;
    constexpr int RIGHT_BOTTOM_H = PANEL_H - RIGHT_3D_H - 6;

    // 子面板: 左栏和中栏各分上下两部分
    constexpr int SUB_H = (PANEL_H - 6) / 2;
}

namespace HudOverlay {
    void drawAll();
}
