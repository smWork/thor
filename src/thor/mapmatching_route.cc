#include <vector>
#include <algorithm>
#include "thor/mapmatching_route.h"
#include <valhalla/midgard/logging.h>

using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::meili;


namespace valhalla {
namespace thor {

  // Form the path from the map-matching results. This path gets sent to
  // TripPathBuilder.
  std::vector<PathInfo> MapMatchingRoute::FormPath(const std::vector<meili::MatchResult>& matched_path,
                                 const std::shared_ptr<sif::DynamicCost>* mode_costing,
                                 baldr::GraphReader& graphreader,
                                 const sif::TravelMode mode) {
    // Set the mode and costing
    mode_ = mode;
    const auto& costing = mode_costing[static_cast<uint32_t>(mode_)];
    // Iterate through the matched path. Form PathInfo - populate elapsed time
    // Return an empty path (or throw exception) if path is not connected.
    float elapsed_time = 0;
    std::vector<PathInfo> path;
    GraphId prior_edge, prior_node;
    const NodeInfo* nodeinfo;
    const DirectedEdge* directededge;
    EdgeLabel pred;
    for (auto& res : matched_path) {
      // Skip node graph types and edges that are the same as the prior edge
      if (res.graphtype() == GraphType::kNode || res.graphid() == prior_edge) {
        continue;
      }

      // Get the directed edge (TODO protect against invalid tile)
      GraphId edge_id = res.graphid();
      const GraphTile* tile = graphreader.GetGraphTile(edge_id);
      directededge = tile->directededge(edge_id);

      // Check if connected to prior edge
      if (prior_edge.Is_Valid()) {
        if (graphreader.GetOpposingEdge(edge_id)->endnode() != prior_node) {
          LOG_ERROR("Matched Path is not connected");
          return {};
        }

        // Get transition cost
        elapsed_time += costing->TransitionCost(directededge, nodeinfo, pred).secs;
      }

      // Get time along the edge. TODO - how do we handle partial distance along
      // the first and last edge?
      elapsed_time += costing->EdgeCost(directededge, nodeinfo->density()).secs;

      // Update the prior_edge and nodeinfo. TODO (protect against invalid tile)
      prior_edge = edge_id;
      prior_node = directededge->endnode();
      const GraphTile* end_tile = graphreader.GetGraphTile(prior_node);
      nodeinfo = end_tile->node(prior_node);

      // Create a predecessor EdgeLabel (for transition costing)
      pred = { kInvalidLabel, edge_id, directededge, { },
               0, 0, directededge->restrictions(),
               directededge->opp_local_idx(), mode, 0 };

      // Add to the PathInfo
      path.emplace_back(mode, elapsed_time, edge_id, 0);
    }
    return path;
  }

}
}
