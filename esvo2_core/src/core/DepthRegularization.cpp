#include <esvo2_core/core/DepthRegularization.h>

namespace esvo2_core
{
namespace core
{
DepthRegularization::DepthRegularization(
  std::shared_ptr<DepthProblemConfig> & dpConfigPtr, size_t numThread)
{
  dpConfigPtr_ = dpConfigPtr;
  numThread_ = std::max((size_t)1, numThread);
  //parameters
  _regularizationRadius = dpConfigPtr_->RegularizationRadius_;//20;//5;//10
  _regularizationMinNeighbours = dpConfigPtr_->RegularizationMinNeighbours_;//32;//8;//16
  _regularizationMinCloseNeighbours = dpConfigPtr_->RegularizationMinCloseNeighbours_;//32;//8;//16
}

DepthRegularization::~DepthRegularization() {}

void DepthRegularization::apply( DepthMap::Ptr& depthMapPtr )
{
  DepthMap &dm = *depthMapPtr.get();

  // The old code built dmTmp point by point, which is the same thing as a copy:
  // every element of dm was copied in, and only the invDepth of valid points
  // was then overwritten. Copying first fixes dmTmp's structure, so the points
  // can be regularized in parallel -- each one reads dm and writes its own cell.
  // Note: SmartGrid has no copy constructor (the implicit one would shallow
  // copy its row pointers), so the copy goes through operator=.
  DepthMap dmTmp(dm.rows(), dm.cols());
  dmTmp = dm;

  std::vector<DepthPoint *> vSrc;
  vSrc.reserve(dm.size());
  for (DepthMap::iterator it = dm.begin(); it != dm.end(); it++)
    vSrc.push_back(&(*it));

  const size_t numThread = std::min(numThread_, std::max((size_t)1, vSrc.size()));
  if (numThread == 1)
    applyRange(dm, dmTmp, vSrc, 0, vSrc.size());
  else
  {
    std::vector<std::thread> threads;
    threads.reserve(numThread);
    const size_t chunk = (vSrc.size() + numThread - 1) / numThread;
    for (size_t i = 0; i < numThread; i++)
    {
      const size_t begin = i * chunk;
      const size_t end = std::min(vSrc.size(), begin + chunk);
      if (begin < end)
        threads.emplace_back(&DepthRegularization::applyRange, this,
                             std::ref(dm), std::ref(dmTmp), std::cref(vSrc), begin, end);
    }
    for (auto &t : threads)
      if (t.joinable())
        t.join();
  }

  //transfer the result
  dm = dmTmp;
}

void DepthRegularization::applyRange(
  DepthMap & dm, DepthMap & dmTmp,
  const std::vector<DepthPoint *> & vSrc, size_t begin, size_t end )
{
  // Reused across points: getNeighbourhood reserves (2r+1)^2 pointers, which at
  // radius 20 is a 13 kB allocation per point, tens of thousands per cycle.
  std::vector<DepthPoint *> neighbours, closeNeighbours;

  for (size_t idx = begin; idx < end; idx++)
  {
    DepthPoint *pSrc = vSrc[idx];
    if (!pSrc->valid())
      continue;
    DepthPoint &newDp = dmTmp.get(pSrc->row(), pSrc->col());

    //get the valid neighbourhood pixels
    neighbours.clear();
    dm.getNeighbourhood(pSrc->row(), pSrc->col(), _regularizationRadius, neighbours);

    bool isSet = false;
    if (neighbours.size() > _regularizationMinNeighbours)
    {
      //find close neighbours (will include this point)
      closeNeighbours.clear();
      // diff < 2*sqrt(var) is diff^2 < 4*var for non-negative diff, which drops
      // a sqrt per neighbour -- tens of millions of them per cycle at radius 20.
      const double fourVarSrc = 4.0 * pSrc->variance();
      const double invDepthSrc = pSrc->invDepth();
      for (size_t i = 0; i < neighbours.size(); i++)
      {
        if (neighbours[i]->valid())
        {
          const double diff = invDepthSrc - neighbours[i]->invDepth();
          const double diff2 = diff * diff;
          if (diff2 < fourVarSrc ||
              diff2 < 4.0 * neighbours[i]->variance())
            closeNeighbours.push_back(neighbours[i]);
        }
      }
      // regularizationMinCloseNeighbours is larger than the fusion's applied region (namely 4 pixels in my implementation)
      if (closeNeighbours.size() > _regularizationMinCloseNeighbours)
      {
        double statisticalMean = 0.0;
        if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "l2") == 0)
        {
          //compute statistical average
          double totalInvVariances = 0.0;
          for (size_t i = 0; i < closeNeighbours.size(); i++)
            totalInvVariances += 1.0 / closeNeighbours[i]->variance();
          for (size_t i = 0; i < closeNeighbours.size(); i++)
            statisticalMean += closeNeighbours[i]->invDepth() *
                               (1.0 / closeNeighbours[i]->variance()) / totalInvVariances;
        }
        else if(strcmp(dpConfigPtr_->LSnorm_.c_str(), "Tdist") == 0)
        {
          double nu_post = closeNeighbours[0]->nu();
          double invDepth_post = closeNeighbours[0]->invDepth();
          double scale2_post = closeNeighbours[0]->scaleSquared();

          for (size_t i = 1; i < closeNeighbours.size(); i++)
          {
            double nu_prior = nu_post;
            double invDepth_prior = invDepth_post;
            double scale2_prior = scale2_post;

            double nu_obs = closeNeighbours[i]->nu();
            double invDepth_obs = closeNeighbours[i]->invDepth();
            double scale2_obs = closeNeighbours[i]->scaleSquared();

            nu_post = std::min(nu_prior, nu_obs);
            invDepth_post = (scale2_obs * invDepth_prior + scale2_prior * invDepth_obs) / (scale2_obs + scale2_prior);
            scale2_post = (nu_post + (invDepth_prior - invDepth_obs) * (invDepth_prior - invDepth_obs) / (scale2_prior + scale2_obs)) /
              (nu_post + 1) * (scale2_prior * scale2_obs) / (scale2_prior + scale2_obs);
          }
          statisticalMean = invDepth_post;
        }
        else
        {
          LOG(INFO) << "(Regularization) Wrong dpConfiguration is provided.";
          exit(-1);
        }

        //set the statistical average (everything else is simply copied)
        newDp.invDepth() = statisticalMean;
        isSet = true;
      }
    }

    if (!isSet)
      newDp.invDepth() = -1.0;
  }
}

}// core
}// esvo2_core
