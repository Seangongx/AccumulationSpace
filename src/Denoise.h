#ifndef DENOISE_H
#define DENOISE_H

#include <string>
#include <utility>

//文件
static char* files[]{"Balls.obj", "Bunny_head.obj", "Cat_head.obj",
                     "David328.obj", "Nefertiti_face.obj"};

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
  int pImplicitIterations = 1;
  int pExplicitIterations = 1;

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

struct DenoiseData {
  //explicit MeshViewerData(pmp::SurfaceMesh& mesh) : minimal_surf_(mesh), curvature_(mesh) {}

  //pmp_pupa::MinimalAreaSurface minimal_surf_;
  //pmp_pupa::SurfaceCurvature curvature_;

  int explicit_iterations_{400};
  int explicit_iterations_prev_{400};
  bool explicit_running_{false};

  int implicit_iterations_{2};
  int implicit_iterations_prev_{2};
  bool implicit_running_{false};

  float lambda_{0.2};

  bool boundary_smoothing_{false};
};

// void do_denoise_processing() {
//   if (data_ == nullptr)
//     data_ = std::make_shared<MeshViewerData>(mesh_);

//   if (data_->explicit_running_ && data_->explicit_iterations_ > 0) {
//     for (int i = 0; i < 10 && data_->explicit_iterations_ > 0; i++) {
//       data_->minimal_surf_.explicit_iterate(data_->lambda_,
//                                             data_->boundary_smoothing_);
//       data_->explicit_iterations_--;
//     }
//     update_mesh();
//   } else if (data_->explicit_running_) {
//     data_->explicit_running_ = false;
//     data_->explicit_iterations_ = data_->explicit_iterations_prev_;
//   }

//   if (data_->implicit_running_ && data_->implicit_iterations_ > 0) {
//     for (int i = 0; i < 10 && data_->implicit_iterations_ > 0; i++) {
//       data_->minimal_surf_.implicit_iterate(data_->lambda_);
//       data_->implicit_iterations_--;
//     }
//     update_mesh();
//   } else if (data_->implicit_running_) {
//     data_->implicit_running_ = false;
//     data_->implicit_iterations_ = data_->implicit_iterations_prev_;
//   }
// }

#endif  //DENOISE_H