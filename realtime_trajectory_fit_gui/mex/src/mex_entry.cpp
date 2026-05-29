#include "types.h"
#include "utils.h"
#include "robot_model.h"
#include "collision_checker.h"
#include "trajectory_solver.h"

#include "mex.h"
#include "matrix.h"

#include <string>
#include <vector>

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
  using namespace rtfg;

  if (nrhs != 1 || !mxIsStruct(prhs[0])) {
    fail("rtfg_solver_mex:InvalidInput", "Usage: out = rtfg_solver_mex(inputStruct)");
  }
  if (nlhs > 1) {
    fail("rtfg_solver_mex:InvalidOutput", "rtfg_solver_mex returns one struct output");
  }

  try {
    const mxArray* in = prhs[0];
    auto getField = [&](const char* name) -> const mxArray* {
      const mxArray* f = mxGetField(in, 0, name);
      if (!f) {
        fail("rtfg_solver_mex:MissingField",
             std::string("Missing input field: ") + name);
      }
      return f;
    };

    std::string scene_urdf = mxArrayToStdString(getField("sceneUrdf"));
    std::string base_link = mxArrayToStdString(getField("baseLink"));
    std::string tip_link = mxArrayToStdString(getField("tipLink"));
    Eigen::VectorXd current_q = readRowVector(getField("currentQ"));
    Eigen::VectorXd initial_q = readRowVector(getField("initialQ"));
    std::vector<Mat4> target_tforms = readTformStack(getField("targetTforms"));
    std::vector<std::string> segment_names = readCellstrVector(getField("segmentNames"));
    std::vector<BasinBox> basin_boxes = readBasinBoxes(getField("basinBoxes"));
    double clearance_threshold = mxGetScalar(getField("clearanceThreshold"));

    SolverConfig cfg;
    cfg.clearance_threshold = clearance_threshold;

    RobotModel robot = loadRobotModel(scene_urdf, base_link, tip_link);
    TrajectoryResult result =
        solveTrajectory(robot, basin_boxes, cfg, target_tforms, segment_names,
                        current_q, initial_q);

    const char* fields[] = {"anchorQSeries",   "playbackQSeries", "tcpPath",
                            "collisionResults", "previewMetrics",  "progressEvents"};
    plhs[0] =
        mxCreateStructMatrix(1, 1, sizeof(fields) / sizeof(fields[0]), fields);
    mxSetField(plhs[0], 0, "anchorQSeries", eigenMatrixToMx(result.anchor_q));
    mxSetField(plhs[0], 0, "playbackQSeries", eigenMatrixToMx(result.playback_q));
    mxSetField(plhs[0], 0, "tcpPath", eigenMatrixToMx(result.tcp_path));
    mxSetField(plhs[0], 0, "collisionResults", buildCollisionResultsStruct(result));
    mxSetField(plhs[0], 0, "previewMetrics", buildMetricsStruct(result));
    mxSetField(plhs[0], 0, "progressEvents",
               buildProgressStruct(result.progress_events));
  } catch (const std::exception& ex) {
    fail("rtfg_solver_mex:RuntimeError", ex.what());
  } catch (...) {
    fail("rtfg_solver_mex:UnknownError", "Unknown exception in rtfg_solver_mex");
  }
}
