#ifndef ESVO2_CORE_CORE_DEPTHREGULARIZATION_H
#define ESVO2_CORE_CORE_DEPTHREGULARIZATION_H

#include <esvo2_core/container/DepthMap.h>
#include <esvo2_core/core/DepthProblem.h>
#include <memory>
#include <thread>
#include <vector>
namespace esvo2_core
{
using namespace container;
namespace core
{
class DepthRegularization
{
public:
  typedef std::shared_ptr<DepthRegularization> Ptr;

  DepthRegularization(std::shared_ptr<DepthProblemConfig> & dpConfigPtr, size_t numThread = 1);
  virtual ~DepthRegularization();

  void apply( DepthMap::Ptr & depthMapPtr );

private:
  // Regularizes the points in [begin, end) of vSrc, reading depthMap and
  // writing the matching cells of dmOut. Each point writes only its own cell.
  void applyRange( DepthMap & depthMap, DepthMap & dmOut,
                   const std::vector<DepthPoint *> & vSrc, size_t begin, size_t end );
  size_t numThread_;
  std::shared_ptr<DepthProblemConfig> dpConfigPtr_;
  size_t _regularizationRadius;
  size_t _regularizationMinNeighbours;
  size_t _regularizationMinCloseNeighbours;
};
}// core
}// esvo2_core

#endif //ESVO2_CORE_CORE_DEPTHREGULARIZATION_H