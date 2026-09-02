#pragma once
#include "RobotModel.h"

namespace SceneRenderer {
    void init();
    void draw3D();
    void drawHud();
    RobotModel& getRobotModel();
}
