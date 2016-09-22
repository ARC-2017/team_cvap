#include <model_matcher.h>
#include <GL/gl.h>
#include <sys/time.h>
#include <cstdio>
#include <stdexcept>
#include <H5Cpp.h>
#include <hdf5_file.h>
#include <utilities.h>
#include <Eigen/Dense>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/ros.h>

ModelMatcher::ModelMatcher(
        int n_cols, int n_rows, float cx, float cy,
        float fx, float fy, int device_id, bool debug, int vec_size, int num_iter_ransac, int min_inliers,
        int min_inliers_redetect, bool use_previous_detection)
    : _running{true}, _n_objects{0}, _n_objects_separate_views{0}, _num_iter_ransac{num_iter_ransac}, _min_inliers(min_inliers), _debug(debug),
      _min_inliers_redetect(min_inliers_redetect), _use_previous_detection(use_previous_detection),
      _max_matches{50000}, _DESCRIPTOR_LENGTH{128},
      _siftEngine{std::unique_ptr<SiftGPU>{new SiftGPU()}},
      _matcherEngine{
          std::unique_ptr<SiftMatchGPU>{new SiftMatchGPU(4096 * vec_size)}} {
    _camera_mat.create(3, 3, CV_32F);
    updateCalibration(n_cols, n_rows, cx, cy, fx, fy);

    char device_id_str[2];
    sprintf(device_id_str, "%d", device_id);

    // allow for full hd with upscaling (2*1920 = 3840)
    // allow for 3K with upscaling (2*2304 = 4608)
    const char *argv_template[] = {"-m",          "-fo",   "-1",    "-s",
                                   "-v",          "0",     "-pack", "-cuda",
                                   device_id_str, "-maxd", "4608"};
    int argc = sizeof(argv_template) / sizeof(char *);

    char *argv[argc];
    for (int i = 0; i < argc; i++)
        argv[i] = strdup(argv_template[i]);

    _siftEngine->ParseParam(argc, argv);

    for (int i = 0; i < argc; i++)
        free(argv[i]);

    if (_siftEngine->CreateContextGL() != SiftGPU::SIFTGPU_FULL_SUPPORTED)
        throw std::runtime_error("ModelMatcher::ModelMatcher: SiftGPU cannot create GL contex\n");

    _matcherEngine->VerifyContextGL();
    _match_buffer.resize(_max_matches * 2);
}

void ModelMatcher::updateCalibration(int n_cols, int n_rows,
                                     float cx,
                                     float cy,
                                     float fx,
                                     float fy) {
    _n_cols = n_cols;
    _n_rows = n_rows;

    // probably wrong!
    _camera_mat.at<float>(0, 0) = fx;
    _camera_mat.at<float>(0, 1) = 0.f;
    _camera_mat.at<float>(0, 2) = cx - _n_cols / 2.0;

    _camera_mat.at<float>(1, 0) = 0.f;
    _camera_mat.at<float>(1, 1) = fy;
    _camera_mat.at<float>(1, 2) = cy - _n_rows / 2.0;

    _camera_mat.at<float>(2, 0) = 0.f;
    _camera_mat.at<float>(2, 1) = 0.f;
    _camera_mat.at<float>(2, 2) = 1.f;
}

void ModelMatcher::addModel(const char *obj_filename) {
    _n_objects++;

    // Read from HDF5-file
    std::cout<<"Reading object from "<<obj_filename<<std::endl;
    std::vector<float> descriptors, positions;
    std::vector<int> d_size, p_size;
    util::HDF5File f(obj_filename);
    f.readArray("descriptors", descriptors, d_size);
    f.readArray("positions", positions, p_size);

    if (d_size.size() != 2)
        throw std::runtime_error("ModelMatcher::addModel: descriptors "
                                 "field should be 2D\n");
    if (d_size.at(1) != _DESCRIPTOR_LENGTH)
        throw std::runtime_error(std::string(
                                     "ModelMatcher::addModel: descriptors should be " +
                                     std::to_string(_DESCRIPTOR_LENGTH) + "-dimensional\n"));
    if (p_size.size() != 2)
        throw std::runtime_error(
                "ModelMatcher::addModel: positions field should be 2D\n");
    if (p_size.at(1) != 3)
        throw std::runtime_error("ModelMatcher::addModel: positions "
                                 "should be 3-dimensional\n");
    if (d_size.at(0) != p_size.at(0))
        throw std::runtime_error("ModelMatcher::addModel: different "
                                 "number of positions and descriptors\n");

    ModelAssets model;
    model.model_size = d_size.at(0);

    for (int i = 0; i < model.model_size; i++) {
        SiftGPU::SiftKeypoint position;
        position.x = positions.at(i * 3);
        position.y = positions.at(i * 3 + 1);
        position.s = positions.at(i * 3 + 2);
        position.o = 0;
        model.positions.push_back(position);
    }

    model.descriptors = descriptors;

    _allModels.push_back(model);
}

void ModelMatcher::addModelSeparateViews(const char *obj_filename){
    _n_objects_separate_views++;
    std::vector<ModelAssets> model_separate_views;

    size_t ext_pos = std::string(obj_filename).find_last_of("_");
    std::string h5_folder = obj_filename;
    h5_folder.replace(ext_pos, 9, "");

    if (!boost::filesystem::is_directory(boost::filesystem::path(h5_folder))) {
        throw std::runtime_error("ModelMatcher::addModelSeparateViews: could "
                                 "not find view folder " + h5_folder);
    }

    std::cout<<"Reading object views from "<<h5_folder<<std::endl;
    int max_views = 150;
    int cur_view =0;
    while (cur_view < max_views){
        std::stringstream ss; ss<<h5_folder<<"/"<<cur_view<<".h5";
        if (!boost::filesystem::exists(boost::filesystem::path(ss.str()))) {
            break;
        }

        std::vector<float> descriptors, positions;
        std::vector<int> d_size, p_size;
        util::HDF5File f(ss.str());
        f.readArray("descriptors", descriptors, d_size);
        f.readArray("positions", positions, p_size);

        if (d_size.size() != 2)
            throw std::runtime_error("ModelMatcher::addModel: descriptors "
                                     "field should be 2D\n");
        if (d_size.at(1) != _DESCRIPTOR_LENGTH)
            throw std::runtime_error(std::string(
                                         "ModelMatcher::addModel: descriptors should be " +
                                         std::to_string(_DESCRIPTOR_LENGTH) + "-dimensional\n"));
        if (p_size.size() != 2)
            throw std::runtime_error(
                    "ModelMatcher::addModel: positions field should be 2D\n");
        if (p_size.at(1) != 3)
            throw std::runtime_error("ModelMatcher::addModel: positions "
                                     "should be 3-dimensional\n");
        if (d_size.at(0) != p_size.at(0))
            throw std::runtime_error("ModelMatcher::addModel: different "
                                     "number of positions and descriptors\n");

        ModelAssets model;
        model.model_size = d_size.at(0);

        for (int i = 0; i < model.model_size; i++) {
            SiftGPU::SiftKeypoint position;
            position.x = positions.at(i * 3);
            position.y = positions.at(i * 3 + 1);
            position.s = positions.at(i * 3 + 2);
            position.o = 0;
            model.positions.push_back(position);
        }

        model.descriptors = descriptors;
        model_separate_views.push_back(model);
        cur_view++;
    }

    if (!model_separate_views.size()){
        throw std::runtime_error("ModelMatcher::addModelSeparateViews: could "
                                 "not find any views in folder " + h5_folder);
    } else {
        std::cout<<"Found "<<model_separate_views.size()<<" views "<<std::endl;
        _allModelsSeparateViews.push_back(model_separate_views);
        _model_best_view_map[_allModelsSeparateViews.size()-1] = -1; // initialize with no "best view"
        _model_previous_inliers_map[_allModelsSeparateViews.size()-1] = -1; // initialize with no "best view"

    }

}

void ModelMatcher::removeAllModels() {
    _allModels.clear();
    _allModelsSeparateViews.clear();
    _n_objects = 0;
    _n_objects_separate_views = 0;
    _model_best_view_map.clear();
    _model_previous_inliers_map.clear();
}

pose::TranslationRotation3D
ModelMatcher::estimatePoseSpecificObject(const cv::Mat &image,
                                         const int object) {
    return estimatePose(image, object);
}


pose::TranslationRotation3D
ModelMatcher::estimatePose(const cv::Mat &image, int object) {

    pose::TranslationRotation3D currPose;

    if (_running) {

        ModelAssets &model = _allModels.at(object);

        _siftEngine->RunSIFT(_n_cols, _n_rows, image.data, GL_LUMINANCE,
                             GL_UNSIGNED_BYTE);

        int n_image_features = _siftEngine->GetFeatureNum();

        std::vector<float> img_feature_descriptors(_DESCRIPTOR_LENGTH *
                                                   n_image_features);
        std::vector<SiftGPU::SiftKeypoint> img_positions(n_image_features);

        _siftEngine->GetFeatureVector(img_positions.data(),
                                      img_feature_descriptors.data());

        const unsigned long long one_d_texture_limit = 134217728;
        if (static_cast<unsigned long long>(model.model_size) *
                static_cast<unsigned long long>(n_image_features) >
                one_d_texture_limit) {

            // shuffling and downsampling image keypoints
            int max_n_image_features = one_d_texture_limit / model.model_size - 1;

            //      std::cout << "going to match " << model.model_size << " model to "
            //                << n_image_features << " image features - max allowed
            // image = "
            //                << max_n_image_features << std::endl;

            std::vector<int> shuffle_inds(n_image_features);
            std::iota(shuffle_inds.begin(), shuffle_inds.end(), 0);
            std::random_shuffle(shuffle_inds.begin(), shuffle_inds.end());

            Eigen::Map<Eigen::VectorXi> shuffle_inds_eig(shuffle_inds.data(),
                                                         n_image_features);
            Eigen::Map<Eigen::MatrixXf> all_keypoints_eig(
                        (float *)img_positions.data(), 4, n_image_features);
            Eigen::Map<Eigen::MatrixXf> all_descriptors_eig(
                        img_feature_descriptors.data(), _DESCRIPTOR_LENGTH, n_image_features);

            Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> p(
                        shuffle_inds_eig);
            all_keypoints_eig = all_keypoints_eig * p;
            all_descriptors_eig = all_descriptors_eig * p;

            n_image_features = max_n_image_features;
        }

        _matcherEngine->SetDescriptors(0, model.model_size,
                                       model.descriptors.data()); // model
        _matcherEngine->SetDescriptors(1, n_image_features,
                                       img_feature_descriptors.data()); // image

        // match and get result
        int num_match = _matcherEngine->GetSiftMatch(
                    _max_matches, (int(*)[2])_match_buffer.data());

        if (_debug){
            std::cout<<"No matches "<<num_match<<std::endl;
        }

        // compute pnp
        std::vector<cv::Point3f> objectPoints;
        std::vector<cv::Point2f> imagePoints;

        for (int i = 0; i < num_match; i++) {
            SiftGPU::SiftKeypoint &objectKey =
                    model.positions[_match_buffer.at(i * 2)];
            SiftGPU::SiftKeypoint &imageKey =
                    img_positions[_match_buffer.at(i * 2 + 1)];

            cv::Point2f imagePoint(imageKey.x - _n_cols / 2,
                                   imageKey.y - _n_rows / 2);
            cv::Point3f objectPoint(objectKey.x, objectKey.y, objectKey.s);

            objectPoints.push_back(objectPoint);
            imagePoints.push_back(imagePoint);
        }

        const float max_dist = 1.0; // 1.0F

        cv::Mat rvec, tvec;
        std::vector<int> inliers_cpu;
        if (objectPoints.size() > 4) {
            cv::solvePnPRansac(objectPoints, imagePoints, _camera_mat,
                               cv::Mat::zeros(1, 8, CV_32F), rvec, tvec, false,
                               _num_iter_ransac, max_dist, objectPoints.size(),
                               inliers_cpu, CV_P3P);
            double T[] = {tvec.at<double>(0, 0), tvec.at<double>(0, 1),
                          tvec.at<double>(0, 2)};
            double R[] = {rvec.at<double>(0, 0), rvec.at<double>(0, 1),
                          rvec.at<double>(0, 2)};
            currPose = pose::TranslationRotation3D(T, R);
        }

        // require at least one inlier for the estimate to be valid (apart from the
        // four points used to estimate pose)
        currPose.setValid(currPose.isFinite() && (inliers_cpu.size() >= 5));
        if (_debug){
            std::cout<<"PnP inliers "<<inliers_cpu.size()<<std::endl;
        }
    }

    return (currPose);
}

std::map<int,pose::TranslationRotation3D> ModelMatcher::estimatePoseAllObjects(const cv::Mat &image){
    std::map<int,pose::TranslationRotation3D> toRet;
    if (_running) {
        // compute max features allowed
        int highest_model_features = -1;
        for (auto model : _allModels){
            if (model.model_size > highest_model_features){
                highest_model_features = model.model_size;
            }
        }
        //      ModelAssets &model = _allModels.at(object);

        _siftEngine->RunSIFT(_n_cols, _n_rows, image.data, GL_LUMINANCE,
                             GL_UNSIGNED_BYTE);

        int n_image_features = _siftEngine->GetFeatureNum();

        std::vector<float> img_feature_descriptors(_DESCRIPTOR_LENGTH *
                                                   n_image_features);
        std::vector<SiftGPU::SiftKeypoint> img_positions(n_image_features);

        _siftEngine->GetFeatureVector(img_positions.data(),
                                      img_feature_descriptors.data());

        const unsigned long long one_d_texture_limit = 134217728;
        if (static_cast<unsigned long long>(highest_model_features) *
                static_cast<unsigned long long>(n_image_features) >
                one_d_texture_limit) {
            std::cout<<"Too many features to match, need to discard some of them"<<std::endl;
            // shuffling and downsampling image keypoints
            int max_n_image_features = one_d_texture_limit / highest_model_features - 1;

            std::vector<int> shuffle_inds(n_image_features);
            std::iota(shuffle_inds.begin(), shuffle_inds.end(), 0);
            std::random_shuffle(shuffle_inds.begin(), shuffle_inds.end());

            Eigen::Map<Eigen::VectorXi> shuffle_inds_eig(shuffle_inds.data(),
                                                         n_image_features);
            Eigen::Map<Eigen::MatrixXf> all_keypoints_eig(
                        (float *)img_positions.data(), 4, n_image_features);
            Eigen::Map<Eigen::MatrixXf> all_descriptors_eig(
                        img_feature_descriptors.data(), _DESCRIPTOR_LENGTH, n_image_features);

            Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> p(
                        shuffle_inds_eig);
            all_keypoints_eig = all_keypoints_eig * p;
            all_descriptors_eig = all_descriptors_eig * p;

            n_image_features = max_n_image_features;
        }

        // match all objects
        for (size_t k=0; k<_n_objects; k++){
            if (_debug){
                std::cout<<"Matching object ------ "<<k<<std::endl;
            }
            ModelAssets &model = _allModels.at(k);

            _matcherEngine->SetDescriptors(0, model.model_size,
                                           model.descriptors.data()); // model
            _matcherEngine->SetDescriptors(1, n_image_features,
                                           img_feature_descriptors.data()); // image

            // match and get result
            int num_match = _matcherEngine->GetSiftMatch(
                        _max_matches, (int(*)[2])_match_buffer.data()/*, 0.6, 0.9,1*/);

            // compute pnp
            std::vector<cv::Point3f> objectPoints;
            std::vector<cv::Point2f> imagePoints;

            for (int i = 0; i < num_match; i++) {
                SiftGPU::SiftKeypoint &objectKey =
                        model.positions[_match_buffer.at(i * 2)];
                SiftGPU::SiftKeypoint &imageKey =
                        img_positions[_match_buffer.at(i * 2 + 1)];

                cv::Point2f imagePoint(imageKey.x - _n_cols / 2,
                                       imageKey.y - _n_rows / 2);
                cv::Point3f objectPoint(objectKey.x, objectKey.y, objectKey.s);

                objectPoints.push_back(objectPoint);
                imagePoints.push_back(imagePoint);
            }

            const float max_dist = 1.0; // 1.0F

            cv::Mat rvec, tvec;
            std::vector<int> inliers_cpu;
            pose::TranslationRotation3D currPose;
            if (objectPoints.size() > 4) {
                cv::solvePnPRansac(objectPoints, imagePoints, _camera_mat,
                                   cv::Mat::zeros(1, 8, CV_32F), rvec, tvec, false,
                                   _num_iter_ransac, max_dist, objectPoints.size(),
                                   inliers_cpu, CV_P3P);
                double T[] = {tvec.at<double>(0, 0), tvec.at<double>(0, 1),
                              tvec.at<double>(0, 2)};
                double R[] = {rvec.at<double>(0, 0), rvec.at<double>(0, 1),
                              rvec.at<double>(0, 2)};
                currPose = pose::TranslationRotation3D(T, R);
            }
             currPose.setValid(currPose.isFinite() && (inliers_cpu.size() >= 10));
             if (currPose.isValid()){
                 if (_debug){
                    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!! Found valid pose for object !!!!!!!!!!!!!!!!!!!!!!!! "<<k<<"  --- inliers --- "<<inliers_cpu.size()<< std::endl;
                 }
                 toRet[k] = currPose;
             }
        }
    }
    return toRet;
}

std::map<int,pose::TranslationRotation3D> ModelMatcher::estimatePoseAllObjectsSeparateViews(const cv::Mat &image){
    std::map<int,pose::TranslationRotation3D> toRet;
    if (_running) {
        // compute max features allowed
        int highest_model_features = -1;
        for (auto model_views : _allModelsSeparateViews){
            for (auto model : model_views){
                if (model.model_size > highest_model_features){
                    highest_model_features = model.model_size;
                }
            }
        }

        _siftEngine->RunSIFT(_n_cols, _n_rows, image.data, GL_LUMINANCE,
                             GL_UNSIGNED_BYTE);

        int n_image_features = _siftEngine->GetFeatureNum();

        std::vector<float> img_feature_descriptors(_DESCRIPTOR_LENGTH *
                                                   n_image_features);
        std::vector<SiftGPU::SiftKeypoint> img_positions(n_image_features);

        _siftEngine->GetFeatureVector(img_positions.data(),
                                      img_feature_descriptors.data());

        const unsigned long long one_d_texture_limit = 134217728;
        if (static_cast<unsigned long long>(highest_model_features) *
                static_cast<unsigned long long>(n_image_features) >
                one_d_texture_limit) {
            std::cout<<"Too many features to match, need to discard some of them"<<std::endl;
            // shuffling and downsampling image keypoints
            int max_n_image_features = one_d_texture_limit / highest_model_features - 1;

            std::vector<int> shuffle_inds(n_image_features);
            std::iota(shuffle_inds.begin(), shuffle_inds.end(), 0);
            std::random_shuffle(shuffle_inds.begin(), shuffle_inds.end());

            Eigen::Map<Eigen::VectorXi> shuffle_inds_eig(shuffle_inds.data(),
                                                         n_image_features);
            Eigen::Map<Eigen::MatrixXf> all_keypoints_eig(
                        (float *)img_positions.data(), 4, n_image_features);
            Eigen::Map<Eigen::MatrixXf> all_descriptors_eig(
                        img_feature_descriptors.data(), _DESCRIPTOR_LENGTH, n_image_features);

            Eigen::PermutationMatrix<Eigen::Dynamic, Eigen::Dynamic, int> p(
                        shuffle_inds_eig);
            all_keypoints_eig = all_keypoints_eig * p;
            all_descriptors_eig = all_descriptors_eig * p;

            n_image_features = max_n_image_features;
        }

//        _matcherEngine->SetDescriptors(1, n_image_features,
//                                       img_feature_descriptors.data()); // image

        // match all objects
        for (size_t k=0; k<_n_objects_separate_views; k++){
            if (_debug){
                std::cout<<"Matching object ------ "<<k<<std::endl;
            }
            pose::TranslationRotation3D objectPose;
            int best_view = -1;
            int max_inliers = 0;

            pose::TranslationRotation3D currPose;
            // check if we already detected this object -> if yes, try first with the same view (to save some time!!!)
            if (_use_previous_detection && (_model_best_view_map[k] != -1)){
                if ((_model_previous_inliers_map[k] >= _min_inliers_redetect)){
                    auto model = _allModelsSeparateViews[k][_model_best_view_map[k]];
                    std::vector<int> inliers_cpu;
                    currPose = matchFeatures(n_image_features, img_feature_descriptors, img_positions,
                                             model, inliers_cpu);
                    if (currPose.isValid()){
                        max_inliers = inliers_cpu.size();
                        objectPose = currPose;
                        best_view = _model_best_view_map[k];
                        _model_previous_inliers_map[k] = max_inliers;
                    }
                } else {
                    if (_debug){
                        ROS_WARN_STREAM("Previous pose inliers "<<_model_previous_inliers_map[k]<<" is below _min_inliers_redetect threshold." <<_min_inliers_redetect<<"  Iterating through all views.");
                    }
                }
            }

            if (best_view == -1){
                // no pose found so far -> try all the views
                for (size_t j=0; j< _allModelsSeparateViews[k].size(); j++){
                    auto model = _allModelsSeparateViews[k][j];
                    std::vector<int> inliers_cpu;
                    currPose = matchFeatures(n_image_features, img_feature_descriptors, img_positions,
                                             model, inliers_cpu);

                    if (currPose.isValid()){
                        if (inliers_cpu.size() > max_inliers){
                            max_inliers = inliers_cpu.size();
                            objectPose = currPose;
                            best_view = j;
                        }
                    }
                }
            }

            if (best_view != -1){
                if (_debug){
                    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!! Found valid pose for object !!!!!!!!!!!!!!!!!!!!!!!! "<<k<<"  --- inliers --- "<<max_inliers<< std::endl;
                }
                _model_best_view_map[k] = best_view;
                _model_previous_inliers_map[k] = max_inliers;
            } else {
                objectPose.setValid(false); // some random pose, not valid
                _model_best_view_map[k] = -1;
                _model_previous_inliers_map[k] = -1;
            }
            toRet[k] = objectPose;
        }
    }

    return toRet;
}

pose::TranslationRotation3D ModelMatcher::matchFeatures(const int& n_image_features, const std::vector<float>& img_feature_descriptors,
                                                        std::vector<SiftGPU::SiftKeypoint>& img_positions,
                                                        ModelAssets& model,  std::vector<int>& inliers_cpu){

    _matcherEngine->SetDescriptors(1, n_image_features,
                                   img_feature_descriptors.data()); // image

    _matcherEngine->SetDescriptors(0, model.model_size,
                                   model.descriptors.data()); // model

    // match and get result
    int num_match = _matcherEngine->GetSiftMatch(
                _max_matches, (int(*)[2])_match_buffer.data()/*, 0.6, 0.9,0*/);

    // compute pnp
    std::vector<cv::Point3f> objectPoints;
    std::vector<cv::Point2f> imagePoints;

    for (int i = 0; i < num_match; i++) {
        SiftGPU::SiftKeypoint &objectKey =
                model.positions[_match_buffer.at(i * 2)];
        SiftGPU::SiftKeypoint &imageKey =
                img_positions[_match_buffer.at(i * 2 + 1)];

        cv::Point2f imagePoint(imageKey.x - _n_cols / 2,
                               imageKey.y - _n_rows / 2);
        cv::Point3f objectPoint(objectKey.x, objectKey.y, objectKey.s);

        objectPoints.push_back(objectPoint);
        imagePoints.push_back(imagePoint);
    }

    const float max_dist = 1.0; // 1.0F

    cv::Mat rvec, tvec;
    pose::TranslationRotation3D currPose;
    if (objectPoints.size() > 4) {
        cv::solvePnPRansac(objectPoints, imagePoints, _camera_mat,
                           cv::Mat::zeros(1, 8, CV_32F), rvec, tvec, false,
                           _num_iter_ransac, max_dist, objectPoints.size(),
                           inliers_cpu, CV_P3P);
        double T[] = {tvec.at<double>(0, 0), tvec.at<double>(0, 1),
                      tvec.at<double>(0, 2)};
        double R[] = {rvec.at<double>(0, 0), rvec.at<double>(0, 1),
                      rvec.at<double>(0, 2)};
        currPose = pose::TranslationRotation3D(T, R);
    }
    currPose.setValid(currPose.isFinite() && (inliers_cpu.size() >= _min_inliers));

    return currPose;

}

void ModelMatcher::setMinInliers(const int& min_inliers){
    _min_inliers = min_inliers;
}

int ModelMatcher::getMinInliers(){
    return _min_inliers;
}

void ModelMatcher::setMinInliersRedetect(const int& min_inliers_redetect){

}

int ModelMatcher::getMinInliersRedetect(){

}

void ModelMatcher::setUsePreviousDetection(const bool& use_previous_detection){
    _use_previous_detection = use_previous_detection;
}

bool ModelMatcher::getUsePreviousDetection(){
    return _use_previous_detection;
}

void ModelMatcher::setDebug(const bool& debug){
    _debug = debug;
}
