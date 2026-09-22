// Times static BM and the static depth solve, double vs float, over every
// golden cycle, repeated to average out noise. Prints a markdown table.
// usage: bench_mapping_float [golden_dir] [repetitions=5]
// Run on an otherwise idle machine.

#include <cstdio>
#include <esvo2_core/tools/TicToc.h>
#include "golden_harness.h"

using namespace golden;

int main(int argc, char **argv)
{
  const std::string dir = argc > 1 ? argv[1] : goldenDir();
  const int reps = argc > 2 ? std::atoi(argv[2]) : 5;
  const std::vector<std::string> files = tools::listGoldenCycles(dir);
  if (files.empty())
  {
    std::fprintf(stderr, "no golden capture in %s\n", dir.c_str());
    return 1;
  }

  std::vector<double> bm[2], dp[2], mirror;
  size_t events = 0, matches = 0;
  for (int rep = 0; rep < reps; rep++)
    for (const std::string &file : files)
    {
      tools::GoldenConfig cfg;
      tools::GoldenCycle c;
      std::string err;
      if (!tools::readGoldenCycle(file, cfg, c, &err))
      {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
      }
      Offline off(cfg);
      std::unique_ptr<constStampedTimeSurfaceObs> obs = makeObservation(c);
      std::vector<dvs_msgs::Event> evs = makeEvents(c);
      std::vector<dvs_msgs::Event *> ptrs = pointersTo(evs);
      for (int f = 0; f < 2; f++)
      {
        std::unique_ptr<core::EventBM> m = off.makeBM(f == 1);
        std::unique_ptr<core::DepthProblemSolver> s = off.makeSolver(f == 1);
        std::vector<core::EventMatchPair> vEMP;
        std::vector<DepthPoint> vdp;
        tools::TicToc t;
        m->createMatchProblem(obs.get(), nullptr, &ptrs); // float: includes refreshing the mirrors
        m->match_all_HyperThread(vEMP);
        bm[f].push_back(t.toc());
        t.tic();
        s->solve(&vEMP, obs.get(), vdp);
        dp[f].push_back(t.toc());
        if (rep == 0 && f == 0)
        {
          events += ptrs.size();
          matches += vEMP.size();
        }
      }
      tools::TicToc t;
      obs->second.refreshFloatMirrors();
      mirror.push_back(t.toc());
    }

  std::printf("%zu cycles x %d repetitions; %zu candidate events, %zu matches per repetition\n\n",
              files.size(), reps, events, matches);
  std::printf("| stage | double median | double p90 | float median | float p90 | speedup (median) |\n");
  std::printf("|---|---|---|---|---|---|\n");
  const char *names[2] = {"static BM (ms)", "static depth solve (ms)"};
  std::vector<double> *stages[2] = {bm, dp};
  for (int k = 0; k < 2; k++)
  {
    const double d50 = percentile(stages[k][0], 0.5), f50 = percentile(stages[k][1], 0.5);
    std::printf("| %s | %.2f | %.2f | %.2f | %.2f | %.2fx |\n", names[k], d50,
                percentile(stages[k][0], 0.9), f50, percentile(stages[k][1], 0.9), d50 / f50);
  }
  std::printf("| float mirror refresh (ms, inside float BM) | - | - | %.2f | %.2f | - |\n",
              percentile(mirror, 0.5), percentile(mirror, 0.9));
  return 0;
}
