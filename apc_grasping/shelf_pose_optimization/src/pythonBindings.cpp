#include <boost/python.hpp>
#include "apc_grasping/shelf_pose/ShelfPoseOptimizer.h"

using namespace apc_grasping::shelf_pose;

// For overloaded functions we need to do this:
void (ShelfPoseOptimizer::*setScenePublisher)(std::string) = &ShelfPoseOptimizer::setScenePublisher;
void (ShelfPoseOptimizer::*setPosesPublisher)(std::string) = &ShelfPoseOptimizer::setPosesPublisher;

BOOST_PYTHON_MODULE(libshelf_pose_optimization)
{
    using namespace boost::python;
    class_<ShelfPoseOptimizer, boost::noncopyable>("ShelfPoseOptimizer", init<>())
        .def("initialize", &ShelfPoseOptimizer::initialize)
        .def("setScenePublisher", setScenePublisher)
        .def("setPosesPublisher", setPosesPublisher)
        .def("evaluateShelfPose", &ShelfPoseOptimizer::evaluateShelfPose);
    
    class_<PoseRange>("PoseRange")
        .def_readwrite("xmin", &PoseRange::xmin)
        .def_readwrite("ymin", &PoseRange::ymin)
        .def_readwrite("zmin", &PoseRange::zmin)
        .def_readwrite("rxmin", &PoseRange::rxmin)
        .def_readwrite("rymin", &PoseRange::rymin)
        .def_readwrite("rzmin", &PoseRange::rzmin)
        .def_readwrite("xmax", &PoseRange::xmax)
        .def_readwrite("ymax", &PoseRange::ymax)
        .def_readwrite("zmax", &PoseRange::zmax)
        .def_readwrite("rxmax", &PoseRange::rxmax)
        .def_readwrite("rymax", &PoseRange::rymax)
        .def_readwrite("rzmax", &PoseRange::rzmax);

    class_<Arguments>("Arguments")
        .def_readwrite("paramsFromServer", &Arguments::paramsFromServer)
        .def_readwrite("paramName", &Arguments::paramName)
        .def_readwrite("urdf", &Arguments::urdf)
        .def_readwrite("srdf", &Arguments::srdf)
        .def_readwrite("shelfMeshPath", &Arguments::shelfMeshPath)
        .def_readwrite("shelfPoseRange", &Arguments::shelfPoseRange)
        .def_readwrite("numEEFWidthSamples", &Arguments::numEEFWidthSamples)
        .def_readwrite("numEEFHeightSamples", &Arguments::numEEFHeightSamples)
        .def_readwrite("numEEFDepthSamples", &Arguments::numEEFDepthSamples)
        .def_readwrite("numEEFRotSamples", &Arguments::numEEFRotSamples);
}
