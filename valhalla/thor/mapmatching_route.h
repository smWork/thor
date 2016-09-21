#ifndef VALHALLA_THOR_MAPMATCHINGROUTE_H_
#define VALHALLA_THOR_MAPMATCHINGROUTE_H_

#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <memory>

#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphreader.h>
#include <valhalla/baldr/double_bucket_queue.h>
#include <valhalla/sif/dynamiccost.h>
#include <valhalla/sif/edgelabel.h>
#include <valhalla/thor/edgestatus.h>
#include <valhalla/thor/pathinfo.h>
#include <valhalla/meili/match_result.h>


namespace valhalla {
namespace thor {

class MapMatchingRoute {
 public:

  std::vector<PathInfo> FormPath(const std::vector<meili::MatchResult>& matched_path,
                                 const std::shared_ptr<sif::DynamicCost>* mode_costing,
                                 baldr::GraphReader& graphreader, const sif::TravelMode mode);

  protected:

  // Current travel mode
  sif::TravelMode mode_;

};

}
}

#endif  // VALHALLA_THOR_MAPMATCHINGROUTE_H_
