#ifndef __MODEL_MATCHER__H
#define __MODEL_MATCHER__H

#include <memory>
#include <translation_rotation_3d.h>
#include <opencv2/core/core.hpp>
#include <SiftGPU.h>

class ModelMatcher {

public:
    ModelMatcher(int n_cols, int n_rows, float cx, float cy, float fx,
                 float fy, int device_id = 0, bool debug = false, int vec_size = 4, int num_iter_ransac = 10000, int min_inliers = 15,
                 int min_inliers_redetect = 35, bool use_previous_detection = true);

  void updateCalibration(int n_cols, int n_rows, float cx,
                         float cy, float fx, float fy);

  void addModel(const char *obj_filename);

  void addModelSeparateViews(const char *obj_filename);

  void removeAllModels();

  void setMinInliers(const int& min_inliers);
  int getMinInliers();
  void setMinInliersRedetect(const int& min_inliers_redetect);
  int getMinInliersRedetect();
  void setUsePreviousDetection(const bool& use_previous_detection);
  bool getUsePreviousDetection();
  void setDebug(const bool& debug);

  // specify for which object the pose should be estimated
  pose::TranslationRotation3D estimatePoseSpecificObject(const cv::Mat &image,
                                                   const int object);

  std::map<int,pose::TranslationRotation3D> estimatePoseAllObjects(const cv::Mat &image);

  std::map<int,pose::TranslationRotation3D> estimatePoseAllObjectsSeparateViews(const cv::Mat &image);

  int getNumberOfObjects() { return (_n_objects); }
  int getNumberOfObjectsSeparateViews() { return (_n_objects_separate_views); }

  void enable() { _running = true; }
  void disable() { _running = false; }

  struct ModelAssets {
    int model_size;
    std::vector<float> descriptors;
    std::vector<SiftGPU::SiftKeypoint> positions;
  };

private:
  pose::TranslationRotation3D estimatePose(const cv::Mat &image, int object = 0);
  pose::TranslationRotation3D matchFeatures(const int& n_image_features, const std::vector<float>& img_feature_descriptors,
                                            std::vector<SiftGPU::SiftKeypoint>& img_positions,
                                            ModelAssets& model,
                                            std::vector<int>& inliers_cpu);

  bool _running;
  bool _debug;
  bool _use_previous_detection;

  cv::Mat _camera_mat;
  int _n_rows, _n_cols;

  const std::unique_ptr<SiftGPU> _siftEngine;
  const std::unique_ptr<SiftMatchGPU> _matcherEngine;
  const int _DESCRIPTOR_LENGTH;
  const int _num_iter_ransac;


  std::vector<ModelAssets> _allModels;
  std::vector<std::vector<ModelAssets>> _allModelsSeparateViews;

  int _n_objects;
  int _n_objects_separate_views;
  int _min_inliers;
  int _min_inliers_redetect;
  std::map<int, int> _model_best_view_map;
  std::map<int, int> _model_previous_inliers_map;

  std::vector<int> _match_buffer;
  const int _max_matches;
};
#endif
