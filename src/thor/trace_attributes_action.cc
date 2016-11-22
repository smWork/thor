#include <vector>
#include <functional>
#include <string>
#include <stdexcept>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <sstream>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/constants.h>
#include <valhalla/baldr/json.h>
#include <valhalla/baldr/geojson.h>
#include <valhalla/baldr/errorcode_util.h>
#include <valhalla/proto/trippath.pb.h>

#include <prime_server/prime_server.hpp>

#include "thor/service.h"
#include "thor/expandfromnode.h"

using namespace prime_server;
using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::odin;
using namespace valhalla::thor;


namespace {
  const headers_t::value_type CORS { "Access-Control-Allow-Origin", "*" };
  const headers_t::value_type JSON_MIME { "Content-type", "application/json;charset=utf-8" };
  const headers_t::value_type JS_MIME { "Content-type", "application/javascript;charset=utf-8" };


  json::MapPtr serialize(const boost::optional<std::string>& id, valhalla::odin::TripPath trip_path) {
    //lets get some edge attributes
    json::ArrayPtr edges = json::array({});
      if (trip_path.node().size() > 0) {
        for(const auto& node : trip_path.node()){
          edges->push_back(static_cast<uint64_t>(node.edge().id()));
          edges->push_back(static_cast<uint64_t>(node.edge().base_data_id()));
          edges->push_back(json::fp_t{node.edge().weighted_grade()});
          edges->push_back(static_cast<uint64_t>(node.edge().max_upward_grade()));
          edges->push_back(static_cast<uint64_t>(node.edge().max_downward_grade()));
          LOG_INFO(" edge id::" + std::to_string(node.edge().id()));
          LOG_INFO(" edge base_data_id::" + std::to_string(node.edge().base_data_id()));
          LOG_INFO(" edge weighted_grade::" + std::to_string(node.edge().weighted_grade()));
          LOG_INFO(" edge max_upward_grade::" + std::to_string(node.edge().max_upward_grade()));
          LOG_INFO(" edge max_downward_grade::" + std::to_string(node.edge().max_downward_grade()));
        }
      }
      auto json = json::map({
        {"trace_attributes", edges}      });
      //  json->emplace("edges", json::array({edges}));
        if (id)
        json->emplace("id", *id);
      return json;
    }
}

namespace valhalla {
namespace thor {

/*
 * The trace_attributes action takes a GPS trace or latitude, longitude positions
 * from a portion of an existing route and returns detailed attribution along the
 * portion of the route. This includes details for each section of road along the
 * path as well as any intersections along the path.
 */
worker_t::result_t thor_worker_t::trace_attributes(
    const boost::property_tree::ptree &request,
    const std::string &request_str, http_request_t::info_t& request_info) {
  parse_locations(request);
  parse_shape(request);
  parse_costing(request);

  //get time for start of request
  auto s = std::chrono::system_clock::now();
  worker_t::result_t result { true  };
  // Forward the original request
  result.messages.emplace_back(request_str);

  // If the exact points from a prior route that was run agains the Valhalla road network,
  //then we can traverse the exact shape to form a path by using edge-walking algorithm
  if (!route_match_get_attributes(result)) {
    //If no Valhalla route match, then use meili map matching
    //to match to local route network. No shortcuts are used and detailed
    //information at every intersection becomes available.
    map_match(result);
  }
  //result.messages.emplace_back(result);
  json::MapPtr json;
  //auto id = attributes.get_optional<std::string>("id");
  //serialize output to Thor
  json = serialize(nullptr, trip_path);


  //jsonp callback if need be
  std::ostringstream stream;
  auto jsonp = request.get_optional<std::string>("jsonp");
  if (jsonp)
    stream << *jsonp << '(';
  stream << *json;
  if (jsonp)
    stream << ')';

  // Get processing time for thor
  auto e = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> elapsed_time = e - s;
  // TODO determine what to log
  //log request if greater than X (ms)
  if (!request_info.do_not_track && (elapsed_time.count() / correlated.size()) > long_request) {
    LOG_WARN("thor::trace_attributes elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
    LOG_WARN("thor::trace_attributes exceeded threshold::"+ request_str);
    midgard::logging::Log("valhalla_thor_long_request_trace_attributes", " [ANALYTICS] ");
  }
  http_response_t response(200, "OK", stream.str(), headers_t{CORS, jsonp ? JS_MIME : JSON_MIME});
  response.from_info(request_info);
 // worker_t::result_t result{false};
  result.messages.emplace_back(response.to_string());
  return result;
}
}

// Form the path from the map-matching results. This path gets sent to TripPathBuilder.
// PathInfo is primarily a list of edge Ids but it also include elapsed time to the end
// of each edge. We will need to use the existing costing method to form the elapsed time
// the path. We will start with just using edge costs and will add transition costs.

/*
 * Returns true if an exact route match using an “edge-walking” algorithm.
 * This is for use when the input shape is exact shape from a prior Valhalla route.
 * This will walk the input shape and compare to Valhalla edge’s end node positions to
 * form the list of edges.
 *
 */

bool thor_worker_t::route_match_get_attributes(worker_t::result_t& result) {
  std::vector<PathInfo> path_infos;
  if (route_match(path_infos)) {
    // Empty through location list
    std::vector<baldr::PathLocation> through_loc;

    // Form the trip path based on mode costing, origin, destination, and path edges
    trip_path = thor::TripPathBuilder::Build(reader, mode_costing,
                                                  path_infos,
                                                  correlated.front(),
                                                  correlated.back(),
                                                  through_loc);

    result.messages.emplace_back(trip_path.SerializeAsString());
    LOG_INFO(">>>>> route_match SUCCESS!!");
    return true;
  }
  return false;
}

bool thor_worker_t::route_match_get_attributes(std::vector<PathInfo>& path_infos) {
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
  thor::ExpandFromNode expandfromnode;
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
      if (expandfromnode.FormPath(mode_costing, mode, reader, shape, index,
                                        end_node_tile, de->endnode(),
                                        end_edge_start_node, prev_edge_label,
                                        elapsed_time, path_infos, false)) {
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
}
