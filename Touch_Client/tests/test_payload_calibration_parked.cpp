// 本文件【不是一个独立的测试套件】—— 它是一个编译开关。
//
// 存在的理由: test_payload_calibration 里有一条【按设计红着】的用例
//   (test_runtime_consistency_guard_replay: 回放四份没有参考量列的 09-19 夹具),
//   它红着就接不进测试床 (测试床退出码 = FAILED 数 ⇒ 整床永远非零); 而删掉它就是把一条
//   未验证的断言【静默消失】。⇒ 拆成两个 exe: 默认变体跑其余全部 (接进测试床), 本变体只跑它。
//
// 为什么用 #include 而不是把 helper 抽成头文件: 主文件 5600+ 行, 那条用例依赖 t6Load /
//   t6LoadAttempt / t6LocalModel / t6SigmaA / t6AnnounceRefSource / MG_COLSCAN 等一堆
//   file-static 与常量。抽头文件要动那些 helper 的边界 (风险), 而 #include 一行就把
//   "零重复代码" 做到了。
//   ⇒ 本文件【有意】只有下面三行 —— 别把它当笔误, 也别顺手往里加第二个套件。
#define PC_PARKED_REPLAY_ONLY
#include "test_payload_calibration.cpp"
