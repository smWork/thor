#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/geojson.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/baldr/errorcode_util.h>
#include <valhalla/meili/map_matcher.h>
#include <valhalla/proto/trippath.pb.h>

#include "thor/service.h"
#include "thor/mapmatching_route.h"
#include "thor/trippathbuilder.h"

using namespace valhalla;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::meili;
using namespace valhalla::thor;

namespace {
const headers_t::value_type CORS { "Access-Control-Allow-Origin", "*" };
const headers_t::value_type JSON_MIME { "Content-type",
    "application/json;charset=utf-8" };
const headers_t::value_type JS_MIME { "Content-type",
    "application/javascript;charset=utf-8" };
}

namespace valhalla {
namespace thor {

worker_t::result_t thor_worker_t::trace_route(
    const boost::property_tree::ptree &request,
    const std::string &request_str) {
  parse_locations(request);
  parse_shape(request);
  parse_costing(request);

  //get time for start of request
  auto s = std::chrono::system_clock::now();
  worker_t::result_t result { true };
  // Forward the original request
  result.messages.emplace_back(request_str);

  // Traverse the exact shape to form a path
  if (!form_path_from_shape(result)) {
    // If no path is formed then call meili
    map_match(result);
  }

  // Get processing time for thor
  auto e = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> elapsed_time = e - s;
  LOG_INFO(">>>>> trace_route time(ms)=" + std::to_string(elapsed_time.count()));
  // TODO determine what to log
  //log request if greater than X (ms)
//  if (!header_dnt && (elapsed_time.count() / correlated.size()) > long_request) {
//    LOG_WARN("thor::route trip_path elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
//    LOG_WARN("thor::route trip_path exceeded threshold::"+ request_str);
//    midgard::logging::Log("valhalla_thor_long_request_route", " [ANALYTICS] ");
//  }

  return result;
}

bool thor_worker_t::form_path_from_shape(worker_t::result_t& result) {
  std::vector<PathInfo> path_infos;
  if (form_path_from_shape(path_infos)) {
    // Empty through location list
    std::vector<baldr::PathLocation> through_loc;

    // Form the trip path based on mode costing, origin, destination, and path edges
    auto trip_path = thor::TripPathBuilder::Build(reader, mode_costing,
                                                  path_infos,
                                                  correlated.front(),
                                                  correlated.back(),
                                                  through_loc);
    result.messages.emplace_back(trip_path.SerializeAsString());

    LOG_INFO(">>>>> form_path_from_shape SUCCESS!!");

    return true;
  }
  return false;
}

bool thor_worker_t::form_path_from_shape(std::vector<PathInfo>& path_infos) {
  float elapsed_time = 0.f;

  // Process and validate begin edge
  const PathLocation::PathEdge* begin_path_edge = find_begin_edge();
  if ((begin_path_edge == nullptr) || !(begin_path_edge->id.Is_Valid())) {
    throw std::runtime_error("Invalid begin edge id");
  }
  const GraphTile* begin_edge_tile = reader.GetGraphTile(begin_path_edge->id);
  if (begin_edge_tile == nullptr) {
    throw std::runtime_error("Begin tile is null");
  }

  // Process and validate end edge
  const PathLocation::PathEdge* end_path_edge = find_end_edge();
  if ((end_path_edge == nullptr) || !(end_path_edge->id.Is_Valid())) {
    throw std::runtime_error("Invalid end edge id");
  }
  const GraphTile* end_edge_tile = reader.GetGraphTile(end_path_edge->id);
  if (end_edge_tile == nullptr) {
    throw std::runtime_error("End tile is null");
  }

  // Assign the end edge start node
  const GraphId end_edge_start_node = find_start_node(end_path_edge->id);

  // Process directed edge and info
  const DirectedEdge* de = begin_edge_tile->directededge(begin_path_edge->id);
  const GraphTile* end_node_tile = reader.GetGraphTile(de->endnode());
  if (begin_edge_tile == nullptr) {
    throw std::runtime_error("End node tile is null");
  }
  PointLL de_end_ll = end_node_tile->node(de->endnode())->latlng();

  // If start and end have the same edge then add and return
  if (begin_path_edge->id == end_path_edge->id) {

    // Update the elapsed time edge cost at single edge
    elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs * (end_path_edge->dist - begin_path_edge->dist);

    // Add single edge
    path_infos.emplace_back(mode, std::round(elapsed_time), begin_path_edge->id, 0);
    return true;
  }

  // Initialize indexes and shape
  size_t index = 0;
  uint32_t shape_length = 0;
  uint32_t de_length = std::round(de->length() * (1 - begin_path_edge->dist)) + 50; // TODO make constant
  EdgeLabel prev_edge_label;

  // Loop over shape to form path from matching edges
  while (index < shape.size()
      && (std::round(shape.at(0).Distance(shape.at(index))) < de_length)) {
    if (shape.at(index).ApproximatelyEqual(de_end_ll)) {

      // Update the elapsed time edge cost at begin edge
      elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs * (1 - begin_path_edge->dist);

      // Add begin edge
      path_infos.emplace_back(mode, std::round(elapsed_time), begin_path_edge->id, 0);

      // Set previous edge label
      prev_edge_label = {kInvalidLabel, begin_path_edge->id, de, {}, 0, 0, mode, 0};

      // Continue walking shape to find the end edge...
      if (expand_from_node(index, end_node_tile, de->endnode(), end_edge_start_node,
                           prev_edge_label, elapsed_time, path_infos, false)) {

        // Update the elapsed time based on transition cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->TransitionCost(
            de, end_edge_tile->node(end_edge_start_node), prev_edge_label).secs;

        // Update the elapsed time based on edge cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs * end_path_edge->dist;

        // Add end edge
        path_infos.emplace_back(mode, std::round(elapsed_time), end_path_edge->id, 0);

        return true;
      } else {
        // Did not find end edge - so get out
        return false;
      }
    }
    index++;
  }
  return false;
}

const PathLocation::PathEdge* thor_worker_t::find_begin_edge() const {
  // Iterate through start edges
  for (const auto& edge : correlated.front().edges) {
    // If origin is at a node - skip any inbound edge
    if (edge.end_node()) {
      continue;
    }
    return &edge;  //TODO special case
  }
  return nullptr;
}

const PathLocation::PathEdge* thor_worker_t::find_end_edge() const{
  // Iterate through end edges
  for (const auto& edge : correlated.back().edges) {
    // If destination is at a node - skip any outbound edge
    if (edge.begin_node()) {
      continue;
    }

    return &edge;  //TODO special case
  }
  return nullptr;
}

const GraphId thor_worker_t::find_start_node(const GraphId& edge_id) {
  const GraphTile* tile = reader.GetGraphTile(edge_id);
  if (tile == nullptr) {
    throw std::runtime_error("Tile is null");
  }
  const DirectedEdge* de = tile->directededge(edge_id);

  GraphId opp_edge_id = tile->GetOpposingEdgeId(de);
  const DirectedEdge* opp_de = tile->directededge(opp_edge_id);

  return opp_de->endnode();
}

//Expand from the node
bool thor_worker_t::expand_from_node(size_t& correlated_index,
                                     const GraphTile* tile, const GraphId& node,
                                     const baldr::GraphId& stop_node,
                                     EdgeLabel& prev_edge_label,
                                     float& elapsed_time,
                                     std::vector<PathInfo>& path_infos,
                                     const bool from_transition) {
  // If node equals stop node then we are done expanding
  if (node == stop_node) {
    return true;
  }

  const NodeInfo* node_info = tile->node(node);
  GraphId edge_id(node.tileid(), node.level(), node_info->edge_index());
  const DirectedEdge* de = tile->directededge(node_info->edge_index());
  for (uint32_t i = 0; i < node_info->edge_count(); i++, de++, edge_id++) {
    // Skip shortcuts
    if (de->is_shortcut()) {
      continue;
    }

    // Process transition edge if previous edge was not from a transition
    if (de->trans_down() || de->trans_up()) {
      if (from_transition) {
        continue;
      } else {
        const GraphTile* end_node_tile = reader.GetGraphTile(de->endnode());
        if (end_node_tile == nullptr) {
          continue;
        }
        if (expand_from_node(correlated_index, end_node_tile, de->endnode(),
                             stop_node, prev_edge_label, elapsed_time, path_infos, true)) {
          return true;
        } else {
          continue;
        }
      }
    }

    // Initialize index and shape
    size_t index = correlated_index;
    uint32_t shape_length = 0;
    uint32_t de_length = de->length() + 50; // TODO make constant

    const GraphTile* end_node_tile = reader.GetGraphTile(de->endnode());
    if (end_node_tile == nullptr) {
      continue;
    }
    PointLL de_end_ll = end_node_tile->node(de->endnode())->latlng();

    // Process current edge until shape matches end node
    // or shape length is longer than the current edge
    while (index < shape.size()
        && (std::round(shape.at(correlated_index).Distance(shape.at(index)))
            < de_length)) {
      if (shape.at(index).ApproximatelyEqual(de_end_ll)) {

        // Update the elapsed time based on transition cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->TransitionCost(
            de, node_info, prev_edge_label).secs;

        // Update the elapsed time based on edge cost
        elapsed_time += mode_costing[static_cast<int>(mode)]->EdgeCost(de).secs;

        // Add edge and update correlated index
        path_infos.emplace_back(mode, std::round(elapsed_time), edge_id, 0);

        // Set previous edge label
        prev_edge_label = {kInvalidLabel, edge_id, de, {}, 0, 0, mode, 0};

        // Continue walking shape to find the end edge...
        return (expand_from_node(index, end_node_tile, de->endnode(),
                                 stop_node, prev_edge_label, elapsed_time, path_infos, false));

      }
      index++;
    }
  }
  return false;
}

void thor_worker_t::map_match(worker_t::result_t& result) {

  // Call Meili for map matching to get a collection of pathLocation Edges
  // Create a matcher
  MapMatcher* matcher;
  try {
    matcher = matcher_factory.Create(config);
  } catch (const std::invalid_argument& ex) {
    //return jsonify_error({400, 499}, request_info, std::string(ex.what()));
    throw std::runtime_error(std::string(ex.what()));
  }

  std::vector<Measurement> sequence;
  for (const auto& coord : shape) {
    sequence.emplace_back(coord, gps_accuracy, search_radius);
  }

  // Create the vector of matched path results
  std::vector<meili::MatchResult> results;
  for (size_t i = 0; i < sequence.size(); i++) {
    results = (matcher->OfflineMatch(sequence));
  }
  LOG_INFO("Matched Results from input trace: ");
  for (const auto& result : results) {
    LOG_INFO(
        std::to_string(result.lnglat().first) + ", "
            + std::to_string(result.lnglat().second));
  }

  // Form the path edges based on the matched points
  thor::MapMatchingRoute mapmatching_route;
  std::vector<PathInfo> path_edges = mapmatching_route.FormPath(matcher,
                                                                results,
                                                                mode_costing,
                                                                mode);

  // Set origin and destination from map matching results
  auto first_result_with_state = std::find_if(
      results.begin(), results.end(),
      [](const meili::MatchResult& result) {return result.HasState();});
  auto last_result_with_state = std::find_if(
      results.rbegin(), results.rend(),
      [](const meili::MatchResult& result) {return result.HasState();});
  if ((first_result_with_state != results.end())
      && (last_result_with_state != results.rend())) {
    baldr::PathLocation origin = matcher->mapmatching().state(
        first_result_with_state->stateid()).candidate();
    baldr::PathLocation destination = matcher->mapmatching().state(
        last_result_with_state->stateid()).candidate();

    // Empty through location list
    std::vector<baldr::PathLocation> through_loc;

    // Form the trip path based on mode costing, origin, destination, and path edges
    auto trip_path = thor::TripPathBuilder::Build(matcher->graphreader(),
                                                  mode_costing, path_edges,
                                                  origin, destination,
                                                  through_loc);
    result.messages.emplace_back(trip_path.SerializeAsString());
    delete matcher;
  } else {
    delete matcher;
    throw baldr::valhalla_exception_t { 400, 442 };
  }
}
}
}
