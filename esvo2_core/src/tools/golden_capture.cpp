#include <esvo2_core/tools/golden_capture.h>

#include <algorithm>
#include <cstdio>
#include <dirent.h>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>

namespace esvo2_core
{
namespace tools
{
namespace
{
std::string cycleStem(const std::string &dir, int cycle)
{
  char name[32];
  std::snprintf(name, sizeof(name), "cycle_%05d", cycle);
  return dir + "/" + name;
}

bool fail(std::string *err, const std::string &msg)
{
  if (err)
    *err = msg;
  return false;
}

bool surfaceToPng(const Eigen::MatrixXd &m, const std::string &path, std::string *err)
{
  if (m.size() == 0 || m.minCoeff() < 0 || m.maxCoeff() > 255 ||
      !(m.array() == m.array().round()).all())
    return fail(err, "surface is not integer-valued in [0,255]: " + path);
  cv::Mat d, u8;
  cv::eigen2cv(m, d);
  d.convertTo(u8, CV_8U);
  if (!cv::imwrite(path, u8))
    return fail(err, "cannot write " + path);
  return true;
}

bool pngToSurface(const std::string &path, Eigen::MatrixXd &m, std::string *err)
{
  cv::Mat u8 = cv::imread(path, cv::IMREAD_UNCHANGED);
  if (u8.empty() || u8.type() != CV_8UC1)
    return fail(err, "cannot read an 8-bit image from " + path);
  cv::Mat d;
  u8.convertTo(d, CV_64F);
  cv::cv2eigen(d, m);
  return true;
}
} // namespace

std::vector<std::string> configDifferences(const GoldenConfig &a, const GoldenConfig &b)
{
  std::vector<std::string> d;
  if (a.patch_size_X != b.patch_size_X) d.push_back("patch_size_X");
  if (a.patch_size_Y != b.patch_size_Y) d.push_back("patch_size_Y");
  if (a.BM_min_disparity != b.BM_min_disparity) d.push_back("BM_min_disparity");
  if (a.BM_max_disparity != b.BM_max_disparity) d.push_back("BM_max_disparity");
  if (a.invDepth_min_range != b.invDepth_min_range) d.push_back("invDepth_min_range");
  if (a.invDepth_max_range != b.invDepth_max_range) d.push_back("invDepth_max_range");
  if (a.BM_step != b.BM_step) d.push_back("BM_step");
  if (a.BM_ZNCC_Threshold != b.BM_ZNCC_Threshold) d.push_back("BM_ZNCC_Threshold");
  if (a.PROCESS_EVENT_NUM != b.PROCESS_EVENT_NUM) d.push_back("PROCESS_EVENT_NUM");
  if (a.num_threads != b.num_threads) d.push_back("num_threads");
  if (a.LSnorm != b.LSnorm) d.push_back("LSnorm");
  if (a.Tdist_nu != b.Tdist_nu) d.push_back("Tdist_nu");
  if (a.Tdist_scale != b.Tdist_scale) d.push_back("Tdist_scale");
  if (a.calibInfoDir != b.calibInfoDir) d.push_back("calibInfoDir");
  return d;
}

bool writeGoldenCycle(const std::string &dir, const GoldenConfig &cfg,
                      const GoldenCycle &c, std::string *err)
{
  const std::string stem = cycleStem(dir, c.cycle);
  if (!surfaceToPng(c.TS_left, stem + "_left.png", err) ||
      !surfaceToPng(c.TS_right, stem + "_right.png", err))
    return false;

  cv::FileStorage fs(stem + ".yml", cv::FileStorage::WRITE);
  if (!fs.isOpened())
    return fail(err, "cannot write " + stem + ".yml");
  fs << "cycle" << c.cycle
     << "patch_size_X" << cfg.patch_size_X << "patch_size_Y" << cfg.patch_size_Y
     << "BM_min_disparity" << cfg.BM_min_disparity << "BM_max_disparity" << cfg.BM_max_disparity
     << "invDepth_min_range" << cfg.invDepth_min_range << "invDepth_max_range" << cfg.invDepth_max_range
     << "BM_step" << cfg.BM_step << "BM_ZNCC_Threshold" << cfg.BM_ZNCC_Threshold
     << "PROCESS_EVENT_NUM" << cfg.PROCESS_EVENT_NUM << "num_threads" << cfg.num_threads
     << "LSnorm" << cfg.LSnorm << "Tdist_nu" << cfg.Tdist_nu << "Tdist_scale" << cfg.Tdist_scale
     << "calibInfoDir" << cfg.calibInfoDir;

  cv::Mat pose(1, 7, CV_64F);
  for (int i = 0; i < 4; i++) pose.at<double>(0, i) = c.q_wxyz[i];
  for (int i = 0; i < 3; i++) pose.at<double>(0, 4 + i) = c.position[i];
  fs << "pose_qwxyz_xyz" << pose;

  cv::Mat ev((int)c.events.size(), 4, CV_64F);
  for (int i = 0; i < ev.rows; i++)
  {
    ev.at<double>(i, 0) = c.events[i].x;
    ev.at<double>(i, 1) = c.events[i].y;
    ev.at<double>(i, 2) = c.events[i].sec;
    ev.at<double>(i, 3) = c.events[i].nsec;
  }
  fs << "events" << ev;

  cv::Mat mt((int)c.matches.size(), 9, CV_64F);
  for (int i = 0; i < mt.rows; i++)
  {
    const GoldenMatch &m = c.matches[i];
    const double row[9] = {m.x_left_raw[0], m.x_left_raw[1], m.x_left[0], m.x_left[1],
                           m.x_right[0], m.x_right[1], m.invDepth, m.cost, m.disp};
    for (int k = 0; k < 9; k++) mt.at<double>(i, k) = row[k];
  }
  fs << "matches" << mt;

  cv::Mat dp((int)c.depths.size(), 5, CV_64F);
  for (int i = 0; i < dp.rows; i++)
  {
    const GoldenDepth &d = c.depths[i];
    const double row[5] = {d.x[0], d.x[1], d.invDepth, d.variance, d.residual};
    for (int k = 0; k < 5; k++) dp.at<double>(i, k) = row[k];
  }
  fs << "depths" << dp;
  return true;
}

bool readGoldenCycle(const std::string &yml_path, GoldenConfig &cfg,
                     GoldenCycle &c, std::string *err)
{
  cv::FileStorage fs(yml_path, cv::FileStorage::READ);
  if (!fs.isOpened())
    return fail(err, "cannot read " + yml_path);
  fs["cycle"] >> c.cycle;
  fs["patch_size_X"] >> cfg.patch_size_X;
  fs["patch_size_Y"] >> cfg.patch_size_Y;
  fs["BM_min_disparity"] >> cfg.BM_min_disparity;
  fs["BM_max_disparity"] >> cfg.BM_max_disparity;
  fs["invDepth_min_range"] >> cfg.invDepth_min_range;
  fs["invDepth_max_range"] >> cfg.invDepth_max_range;
  fs["BM_step"] >> cfg.BM_step;
  fs["BM_ZNCC_Threshold"] >> cfg.BM_ZNCC_Threshold;
  fs["PROCESS_EVENT_NUM"] >> cfg.PROCESS_EVENT_NUM;
  fs["num_threads"] >> cfg.num_threads;
  fs["LSnorm"] >> cfg.LSnorm;
  fs["Tdist_nu"] >> cfg.Tdist_nu;
  fs["Tdist_scale"] >> cfg.Tdist_scale;
  fs["calibInfoDir"] >> cfg.calibInfoDir;

  cv::Mat pose, ev, mt, dp;
  fs["pose_qwxyz_xyz"] >> pose;
  if (pose.rows != 1 || pose.cols != 7)
    return fail(err, "bad pose in " + yml_path);
  for (int i = 0; i < 4; i++) c.q_wxyz[i] = pose.at<double>(0, i);
  for (int i = 0; i < 3; i++) c.position[i] = pose.at<double>(0, 4 + i);

  fs["events"] >> ev;
  c.events.clear();
  for (int i = 0; i < ev.rows; i++)
    c.events.push_back({(uint16_t)ev.at<double>(i, 0), (uint16_t)ev.at<double>(i, 1),
                        (uint32_t)ev.at<double>(i, 2), (uint32_t)ev.at<double>(i, 3)});

  fs["matches"] >> mt;
  c.matches.clear();
  for (int i = 0; i < mt.rows; i++)
  {
    GoldenMatch m;
    m.x_left_raw = {mt.at<double>(i, 0), mt.at<double>(i, 1)};
    m.x_left = {mt.at<double>(i, 2), mt.at<double>(i, 3)};
    m.x_right = {mt.at<double>(i, 4), mt.at<double>(i, 5)};
    m.invDepth = mt.at<double>(i, 6);
    m.cost = mt.at<double>(i, 7);
    m.disp = mt.at<double>(i, 8);
    c.matches.push_back(m);
  }

  fs["depths"] >> dp;
  c.depths.clear();
  for (int i = 0; i < dp.rows; i++)
  {
    GoldenDepth d;
    d.x = {dp.at<double>(i, 0), dp.at<double>(i, 1)};
    d.invDepth = dp.at<double>(i, 2);
    d.variance = dp.at<double>(i, 3);
    d.residual = dp.at<double>(i, 4);
    c.depths.push_back(d);
  }

  const std::string stem = yml_path.substr(0, yml_path.size() - 4); // strip ".yml"
  return pngToSurface(stem + "_left.png", c.TS_left, err) &&
         pngToSurface(stem + "_right.png", c.TS_right, err);
}

std::vector<std::string> listGoldenCycles(const std::string &dir)
{
  std::vector<std::string> files;
  DIR *d = opendir(dir.c_str());
  if (!d)
    return files;
  while (dirent *e = readdir(d))
  {
    const std::string name = e->d_name;
    if (name.size() > 10 && name.compare(0, 6, "cycle_") == 0 &&
        name.compare(name.size() - 4, 4, ".yml") == 0)
      files.push_back(dir + "/" + name);
  }
  closedir(d);
  std::sort(files.begin(), files.end());
  return files;
}
} // namespace tools
} // namespace esvo2_core
