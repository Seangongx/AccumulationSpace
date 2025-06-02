#ifndef COMMOM_H
#define COMMON_H

#include <string>
#include <utility>

struct ImguiParams {

  // 滑块/范围参数
  std::pair<int, int> accFilterRange;
  std::pair<int, int> defaultAccRange;

  // Denoise参数
  float pLambda = 0.1f;
  float minLambda = 0.0f;
  float maxLambda = 1.0f;
  int pNoiseMode = 0;
  int pNoiseScale = 0;
  int pExplicitIterations = 1;
  int pExplicitIterations_prev = 1;
  bool pExplicitRunning = false;
  int pImplicitIterations = 1;
  int pImplicitIterations_prev = 1;
  bool pImplicitRunning = false;
  bool pBoundarySmoothing = false;

  // 画刷参数
  float paintRad = 1.0f;
  float noiseLevel = 1.0f;

  // 按钮状态
  bool accBtnPressed0 = false;
  bool accBtnPressed1 = false;
  bool accBtnPressed2 = false;
  bool accBtnPressed3 = false;
  bool accBtnPressed4 = false;
  bool accBtnPressed5 = false;

  // 其他参数
  std::string promptText = "select nothing at the beginning\n";
  // ... 继续添加你需要的参数

  // 可选：参数重置
  void reset() {
    // 重置所有参数为默认值
    *this = ImguiParams();
  }
};

// Default files list
static char* files[]{"Balls.obj", "Bunny_head.obj", "Cat_head.obj",
                     "David328.obj", "Nefertiti_face.obj"};

#endif