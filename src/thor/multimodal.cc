#include <map>
#include <algorithm>
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/errorcode_util.h>
#include <valhalla/midgard/logging.h>
#include "thor/multimodal.h"

using namespace valhalla::baldr;
using namespace valhalla::sif;

namespace {

// Method to get an operator Id from a map of operator strings vs. Id.
uint32_t GetOperatorId(const GraphTile* tile, uint32_t routeid,
            std::unordered_map<std::string, uint32_t>& operators) {
  const TransitRoute* transit_route = tile->GetTransitRoute(routeid);

  // Test if the transit operator changed
  if (transit_route && transit_route->op_by_onestop_id_offset()) {
    // Get the operator name and look up in the operators map
    std::string operator_name =
        tile->GetName(transit_route->op_by_onestop_id_offset());
    auto operator_itr = operators.find(operator_name);
    if (operator_itr == operators.end()) {
      // Operator not found - add to the map
      uint32_t id = operators.size() + 1;
      operators[operator_name] = id;
      return id;
    } else {
      return operator_itr->second;
    }
  }
  return 0;
}

}

namespace valhalla {
namespace thor {

constexpr uint64_t kInitialEdgeLabelCount = 200000;

// Default constructor
MultiModalPathAlgorithm::MultiModalPathAlgorithm()
    : AStarPathAlgorithm(),
      walking_distance_(0) {
}

// Destructor
MultiModalPathAlgorithm::~MultiModalPathAlgorithm() {
  Clear();
}

// Initialize prior to finding best path
void MultiModalPathAlgorithm::Init(const PointLL& origll,
                       const PointLL& destll,
                       const std::shared_ptr<DynamicCost>& costing) {
  // Disable A* for multimodal
  astarheuristic_.Init(destll, 0.0f);

  // Reserve size for edge labels - do this here rather than in constructor so
  // to limit how much extra memory is used for persistent objects
  edgelabels_.reserve(kInitialEdgeLabelCount);

  // Set up lambda to get sort costs
  const auto edgecost = [this](const uint32_t label) {
    return edgelabels_[label].sortcost();
  };

  // Construct adjacency list and edge status.
  // Set bucket size and cost range based on DynamicCost.
  uint32_t bucketsize = costing->UnitSize();
  float range = kBucketCount * bucketsize;
  adjacencylist_.reset(new DoubleBucketQueue(0.0f, range, bucketsize, edgecost));
  edgestatus_.reset(new EdgeStatus());

  // Get hierarchy limits from the costing. Get a copy since we increment
  // transition counts (i.e., this is not a const reference).
  hierarchy_limits_  = costing->GetHierarchyLimits();
}

// Calculate best path using multiple modes (e.g. transit).
std::vector<PathInfo> MultiModalPathAlgorithm::GetBestPath(
            PathLocation& origin, PathLocation& destination,
            GraphReader& graphreader,
            const std::shared_ptr<DynamicCost>* mode_costing,
            const TravelMode mode) {
  // For pedestrian costing - set flag allowing use of transit connections
  // Set pedestrian costing to use max distance. TODO - need for other modes
  const auto& pc = mode_costing[static_cast<uint32_t>(TravelMode::kPedestrian)];
  pc->SetAllowTransitConnections(true);
  pc->UseMaxMultiModalDistance();

  // Check if there no possible path to destination based on mode to the
  // destination - for now assume pedestrian
  // TODO - some means of setting destination mode
  if (!CanReachDestination(destination, graphreader, TravelMode::kPedestrian, pc)) {
    LOG_INFO("Cannot reach destination - too far from a transit stop");
    return { };
  }

  // Set the mode from the origin
  mode_ = mode;
  const auto& costing = mode_costing[static_cast<uint32_t>(mode)];
  const auto& tc = mode_costing[static_cast<uint32_t>(TravelMode::kPublicTransit)];
  bool wheelchair = tc->wheelchair();
  bool bicycle = tc->bicycle();

  // Get maximum transfer distance
  uint32_t max_transfer_distance = costing->GetMaxTransferDistanceMM();

  // For now the date_time must be set on the origin.
  if (!origin.date_time_)
    return { };

  // Initialize - create adjacency list, edgestatus support, A*, etc.
  //Note: because we can correlate to more than one place for a given PathLocation
  //using edges.front here means we are only setting the heuristics to one of them
  //alternate paths using the other correlated points to may be harder to find
  Init(origin.edges.front().projected, destination.edges.front().projected, costing);
  float mindist = astarheuristic_.GetDistance(origin.edges.front().projected);

  // Initialize the origin and destination locations. Initialize the
  // destination first in case the origin edge includes a destination edge.
  SetDestination(graphreader, destination, costing);
  SetOrigin(graphreader, origin, destination, costing);

  uint32_t start_time, localtime, date, dow, day = 0;
  bool date_before_tile = false;
  if (origin.date_time_) {
    // Set route start time (seconds from midnight), date, and day of week
    start_time = DateTime::seconds_from_midnight(*origin.date_time_);
    localtime = start_time;
  }

  bool date_set = false;
  // Find shortest path
  uint32_t blockid, tripid;
  uint32_t nc = 0;       // Count of iterations with no convergence
                         // towards destination
  std::unordered_map<std::string, uint32_t> operators;
  std::unordered_set<uint32_t> processed_tiles;

  const GraphTile* tile;
  while (true) {
    // Allow this process to be aborted
    if(interrupt && (edgelabels_.size() % 5000) == 0)
      (*interrupt)();

    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjacencylist_->pop();
    if (predindex == kInvalidLabel) {
      LOG_ERROR("Route failed after iterations = " +
                     std::to_string(edgelabels_.size()));
      return { };
    }

    // Copy the EdgeLabel for use in costing. Check if this is a destination
    // edge and potentially complete the path.
    EdgeLabel pred = edgelabels_[predindex];
    if (destinations_.find(pred.edgeid()) != destinations_.end()) {
      // Check if a trivial path. Skip if no predecessor and not
      // trivial (cannot reach destination along this one edge).
      if (pred.predecessor() == kInvalidLabel) {
        if (IsTrivial(pred.edgeid(), origin, destination)) {
          return FormPath(predindex);
        }
      } else {
        return FormPath(predindex);
      }
    }

    // Mark the edge as permanently labeled. Do not do this for an origin
    // edge (this will allow loops/around the block cases)
    if (!pred.origin()) {
      edgestatus_->Update(pred.edgeid(), EdgeSet::kPermanent);
    }

    // Check that distance is converging towards the destination. Return route
    // failure if no convergence for TODO iterations
    float dist2dest = pred.distance();
    if (dist2dest < mindist) {
      mindist = dist2dest;
      nc = 0;
    } else if (nc++ > 500000) {
      return {};
    }

    // Get the end node. Skip if tile not found (can happen with
    // regional data sets).
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }

    // Check access at the node
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Set local time. TODO: adjust for time zone.
    uint32_t localtime = start_time + pred.cost().secs;

    // Set a default transfer penalty at a stop (if not same trip Id and block Id)
    Cost transfer_cost = tc->DefaultTransferCost();

    // Get any transfer times and penalties if this is a transit stop (and
    // transit has been taken at some point on the path) and mode is pedestrian
    mode_ = pred.mode();
    bool has_transit = pred.has_transit();
    GraphId prior_stop = pred.prior_stopid();
    uint32_t operator_id = pred.transit_operator();
    if (nodeinfo->type() == NodeType::kMultiUseTransitStop) {

      // Get the transfer penalty when changing stations
      if (mode_ == TravelMode::kPedestrian && prior_stop.Is_Valid() && has_transit) {
        transfer_cost = tc->TransferCost();
      }

      if (processed_tiles.find(tile->id().tileid()) == processed_tiles.end()) {
        tc->AddToExcludeList(tile);
        processed_tiles.emplace(tile->id().tileid());
      }

      //check if excluded.
      if (tc->IsExcluded(tile, nodeinfo))
        continue;

      // Add transfer time to the local time when entering a stop
      // as a pedestrian. This is a small added cost on top of
      // any costs along paths and roads
      if (mode_ == TravelMode::kPedestrian) {
        localtime += transfer_cost.secs;
      }

      // Update prior stop. TODO - parent/child stop info?
      prior_stop = node;

      // we must get the date from level 3 transit tiles and not level 2.  The level 3 date is
      // set when the fetcher grabbed the transit data and created the schedules.
      if (!date_set) {
        date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(*origin.date_time_));
        dow  = DateTime::day_of_week_mask(*origin.date_time_);
        uint32_t date_created = tile->header()->date_created();
        if (date < date_created)
          date_before_tile = true;
        else
          day = date - date_created;

        date_set = true;
      }
    }

    // Allow mode changes at special nodes
    //      bike share (pedestrian <--> bicycle)
    //      parking (drive <--> pedestrian)
    //      transit stop (pedestrian <--> transit).
    // TODO - evaluate how this will work when an edge may have already
    // been visited using a different mode.
    bool mode_change = false;
   /*if (nodeinfo->type() == NodeType::kBikeShare) {
      if (mode_ == TravelMode::kBicycle) {
        mode_ = TravelMode::kPedestrian;
        mode_change = true;
      } else if (mode_ == TravelMode::kPedestrian) {
        mode_ = TravelMode::kBicycle;
        mode_change = true;
      }
    } else if (nodeinfo->type() == NodeType::kParking) {
      if (mode_ == TravelMode::kDrive) {
        mode_ = TravelMode::kPedestrian;
        mode_change = true;
      } else if (mode_ == TravelMode::kPedestrian) {
        mode_ = TravelMode::kDrive;
        mode_change = true;
      }
    }*/

    // Expand from end node.
    uint32_t shortcuts = 0;
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, edgeid++) {
      // Skip shortcuts
      if (directededge->is_shortcut()) {
        continue;
      }

      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo edgestatus = edgestatus_->Get(edgeid);
      if (edgestatus.set() == EdgeSet::kPermanent) {
        continue;
      }

      if (directededge->trans_up() || directededge->trans_down()) {
        // Add the transition edge to the adjacency list and edge labels
        // using the predecessor information. Transition edges have
        // no length.
        AddToAdjacencyList(edgeid, pred.sortcost());
        edgelabels_.emplace_back(predindex, edgeid, directededge->endnode(), pred);
        continue;
      }

      // Reset cost and walking distance
      Cost newcost = pred.cost();
      walking_distance_ = pred.path_distance();

      // If this is a transit edge - get the next departure. Do not check
      // if allowed by costing - assume if you get a transit edge you
      // walked to the transit stop
      tripid  = 0;
      blockid = 0;
      if (directededge->IsTransitLine()) {
        // Check if transit costing allows this edge
        if (!tc->Allowed(directededge, pred, tile, edgeid)) {
          continue;
        }
        //check if excluded.
        if (tc->IsExcluded(tile, directededge))
          continue;

        // Look up the next departure along this edge
        const TransitDeparture* departure = tile->GetNextDeparture(
                    directededge->lineid(), localtime, day, dow, date_before_tile,
                    wheelchair, bicycle);
        if (departure) {
          // Check if there has been a mode change
          mode_change = (mode_ == TravelMode::kPedestrian);

          // Update trip Id and block Id
          tripid  = departure->tripid();
          blockid = departure->blockid();
          has_transit = true;

          // There is no cost to remain on the same trip or valid blockId
          if ( tripid == pred.tripid() ||
              (blockid != 0 && blockid == pred.blockid())) {
            // This departure is valid without any added cost. Operator Id
            // is the same as the predecessor
            operator_id = pred.transit_operator();
          } else {
            if (pred.tripid() > 0) {
              // tripId > 0 means the prior edge was a transit edge and this
              // is an "in-station" transfer. Add a small transfer time and
              // call GetNextDeparture again if we cannot make the current
              // departure.
              // TODO - is there a better way?
              if (localtime + 30 > departure->departure_time()) {
                  departure = tile->GetNextDeparture(directededge->lineid(),
                                localtime + 30, day, dow, date_before_tile,
                                wheelchair, bicycle);
                if (!departure)
                  continue;
              }
            }

            // Get the operator Id
            operator_id = GetOperatorId(tile, departure->routeid(), operators);

            // Add transfer penalty and operator change penalty
            newcost.cost += transfer_cost.cost;
            if (pred.transit_operator() > 0 &&
                pred.transit_operator() != operator_id) {
              // TODO - create a configurable operator change penalty
              newcost.cost += 300;
            }
          }

          // Change mode and costing to transit. Add edge cost.
          mode_ = TravelMode::kPublicTransit;
          newcost += tc->EdgeCost(directededge, departure, localtime);
        } else {
          // No matching departures found for this edge
          continue;
        }
      } else {
        // If current mode is public transit we should only connect to
        // transit connection edges or transit edges
        if (mode_ == TravelMode::kPublicTransit) {
          // Disembark from transit and reset walking distance
          mode_ = TravelMode::kPedestrian;
          walking_distance_ = 0;
          mode_change = true;
        }

        // Regular edge - use the appropriate costing and check if access
        // is allowed. If mode is pedestrian this will validate walking
        // distance has not been exceeded.
        if (!mode_costing[static_cast<uint32_t>(mode_)]->Allowed(
                directededge, pred, tile, edgeid)) {
          continue;
        }

        Cost c = mode_costing[static_cast<uint32_t>(mode_)]->EdgeCost(directededge);
        c.cost *= mode_costing[static_cast<uint32_t>(mode_)]->GetModeWeight();
        newcost += c;

        // Add to walking distance
        if (mode_ == TravelMode::kPedestrian) {
          walking_distance_ += directededge->length();

          // Prevent going from one transit connection directly to another
          // at a transit stop - this is like entering a station and exiting
          // without getting on transit
          if (nodeinfo->type() == NodeType::kMultiUseTransitStop &&
              pred.use()   == Use::kTransitConnection &&
              directededge->use()  == Use::kTransitConnection)
                continue;
        }
      }

      // Add mode change cost or edge transition cost from the costing model
      if (mode_change) {
        // TODO: make mode change cost configurable. No cost for entering
        // a transit line (assume the wait time is the cost)
        ;  //newcost += {10.0f, 10.0f };
      } else {
        newcost += mode_costing[static_cast<uint32_t>(mode_)]->TransitionCost(
               directededge, nodeinfo, pred);
      }

      // If this edge is a destination, subtract the partial/remainder cost
      // (cost from the dest. location to the end of the edge)
      auto p = destinations_.find(edgeid);
      if (p != destinations_.end()) {
        newcost -= p->second;
      }

      // Prohibit entering the same station as the prior.
      if (directededge->use() == Use::kTransitConnection &&
          directededge->endnode() == pred.prior_stopid()) {
        continue;
      }

      // Test if exceeding maximum transfer walking distance
      if (directededge->use() == Use::kTransitConnection &&
          pred.prior_stopid().Is_Valid() &&
          walking_distance_ > max_transfer_distance) {
        continue;
      }

      // Check if edge is temporarily labeled and this path has less cost. If
      // less cost the predecessor is updated and the sort cost is decremented
      // by the difference in real cost (A* heuristic doesn't change). Update
      // trip Id and block Id.
      if (edgestatus.set() == EdgeSet::kTemporary) {
        uint32_t idx = edgestatus.index();
        float dc = edgelabels_[idx].cost().cost - newcost.cost;
        if (dc > 0) {
          float oldsortcost = edgelabels_[idx].sortcost();
          float newsortcost = oldsortcost - dc;
          edgelabels_[idx].Update(predindex, newcost, newsortcost,
                                  walking_distance_, tripid, blockid);
          adjacencylist_->decrease(idx, newsortcost, oldsortcost);
        }
        continue;
      }

      // If this is a destination edge the A* heuristic is 0. Otherwise the
      // sort cost (with A* heuristic) is found using the lat,lng at the
      // end node of the directed edge.
      float dist = 0.0f;
      float sortcost = newcost.cost;
      if (p == destinations_.end()) {
        // Get the end node, skip if the end node tile is not found
        const GraphTile* endtile = (directededge->leaves_tile()) ?
            graphreader.GetGraphTile(directededge->endnode()) : tile;
        if (endtile == nullptr) {
          continue;
        }
        const NodeInfo* endnode = endtile->node(directededge->endnode());
        dist = astarheuristic_.GetDistance(endnode->latlng());
        sortcost += astarheuristic_.Get(dist);
      }

      // Add edge label, add to the adjacency list and set edge status
      AddToAdjacencyList(edgeid, sortcost);
      edgelabels_.emplace_back(predindex, edgeid, directededge,
                    newcost, sortcost, dist, mode_, walking_distance_,
                    tripid, prior_stop, blockid, operator_id, has_transit);
    }
  }
  return {};      // Should never get here
}

// Check if destination can be reached if walking is the last mode. Checks
// if there are any transit stops within maximum walking distance.
// TODO - once auto/bicycle are allowed modes we need to check if parking
// or bikeshare locations are within walking distance.
bool MultiModalPathAlgorithm::CanReachDestination(const PathLocation& destination,
                          GraphReader& graphreader,
                          const TravelMode dest_mode,
                          const std::shared_ptr<DynamicCost>& costing) {
  // Assume pedestrian mode for now
  mode_ = dest_mode;

  // Set up lambda to get sort costs
  const auto edgecost = [this](const uint32_t label) {
    return edgelabels_[label].sortcost();
  };

  // Use a simple Dijkstra method - no need to recover the path just need to
  // make sure we can get to a transit stop within the specified max. walking
  // distance
  uint32_t label_idx = 0;
  uint32_t bucketsize = costing->UnitSize();
  DoubleBucketQueue adjlist(0.0f, kBucketCount * bucketsize, bucketsize, edgecost);
  std::vector<EdgeLabel> edgelabels;
  EdgeStatus edgestatus;

  // Add the opposing destination edges to the priority queue
  for (const auto& edge : destination.edges) {
    // Keep the id and the cost to traverse the partial distance
    float ratio = (1.0f - edge.dist);
    GraphId oppedge = graphreader.GetOpposingEdgeId(edge.id);
    const GraphTile* tile = graphreader.GetGraphTile(oppedge);
    const DirectedEdge* diredge = tile->directededge(oppedge);
    uint32_t length = static_cast<uint32_t>(diredge->length()) * ratio;
    Cost cost = costing->EdgeCost(diredge) * ratio;
    edgelabels.emplace_back(kInvalidLabel, oppedge,
            diredge, cost, cost.cost, 0.0f, mode_, length);
    adjlist.add(label_idx, cost.cost);
    edgestatus.Set(oppedge, EdgeSet::kTemporary, label_idx);
    label_idx++;
  }

  // TODO - we really want to traverse in reverse direction - but since
  // pedestrian access should be the same in either direction we will
  // traverse in a forward direction for now
  const GraphTile* tile;
  while (true) {
    // Get next element from adjacency list. Check that it is valid. An
    // invalid label indicates there are no edges that can be expanded.
    uint32_t predindex = adjlist.pop();
    if (predindex == kInvalidLabel) {
      // Throw an exception so the message is returned in the service
      throw valhalla_exception_t{400, 440};
      return false;
    }

    // Mark the edge as as permanently labeled - copy the EdgeLabel
    // for use in costing
    EdgeLabel pred = edgelabels[predindex];
    edgestatus.Set(pred.edgeid(), EdgeSet::kPermanent, pred.edgeid());

    // Get the end node of the prior directed edge and check access
    GraphId node = pred.endnode();
    if ((tile = graphreader.GetGraphTile(node)) == nullptr) {
      continue;
    }
    const NodeInfo* nodeinfo = tile->node(node);
    if (!costing->Allowed(nodeinfo)) {
      continue;
    }

    // Return true if we reach a transit stop
    if (nodeinfo->type() == NodeType::kMultiUseTransitStop) {
      return true;
    }

    // Expand edges from the node
    GraphId edgeid(node.tileid(), node.level(), nodeinfo->edge_index());
    const DirectedEdge* directededge = tile->directededge(nodeinfo->edge_index());
    for (uint32_t i = 0; i < nodeinfo->edge_count(); i++, directededge++, edgeid++) {
      // Get the current set. Skip this edge if permanently labeled (best
      // path already found to this directed edge).
      EdgeStatusInfo es = edgestatus.Get(edgeid);
      if (es.set() == EdgeSet::kPermanent) {
        continue;
      }

      // Handle transition edges
      if (directededge->trans_up() || directededge->trans_down()) {
        // Add the transition edge to the adjacency list and edge labels
        // using the predecessor information.
        edgelabels.emplace_back(predindex, edgeid, directededge->endnode(), pred);
        adjlist.add(label_idx, pred.sortcost());
        edgestatus.Set(edgeid, EdgeSet::kTemporary, label_idx);
        label_idx++;
        continue;
      }

      // Skip if access is not allowed for this mode
      if (!costing->Allowed(directededge, pred, tile, edgeid)) {
        continue;
      }

      // Get cost
      Cost newcost = pred.cost() + costing->EdgeCost(directededge) +
                     costing->TransitionCost(directededge, nodeinfo, pred);
      uint32_t walking_distance = pred.path_distance() + directededge->length();

      // Check if lower cost path
      if (es.set() == EdgeSet::kTemporary) {
        uint32_t idx = es.index();
        float dc = edgelabels[idx].cost().cost - newcost.cost;
        if (dc > 0) {
          float oldsortcost = edgelabels[idx].sortcost();
          float newsortcost = oldsortcost - dc;
          edgelabels[idx].Update(predindex, newcost, newsortcost,
                                  walking_distance, 0, 0);
          adjlist.decrease(idx, newsortcost, oldsortcost);
        }
        continue;
      }

      // Add edge label, add to the adjacency list and set edge status
      edgelabels.emplace_back(predindex, edgeid, directededge,
                    newcost, newcost.cost, 0.0f, mode_, walking_distance);
      adjlist.add(label_idx, newcost.cost);
      edgestatus.Set(edgeid, EdgeSet::kTemporary, label_idx);
      label_idx++;
    }
  }
  return false;
}

}
}
