#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/geojson.h>
#include <valhalla/baldr/pathlocation.h>
#include <valhalla/meili/map_matcher_factory.h>
#include <valhalla/meili/map_matcher.h>
#include <valhalla/proto/trippath.pb.h>


#include "thor/service.h"
#include "thor/mapmatching_route.h"
#include "thor/trippathbuilder.h"

using namespace valhalla;
using namespace valhalla::meili;
using namespace valhalla::thor;

namespace {
  const headers_t::value_type CORS{"Access-Control-Allow-Origin", "*"};
  const headers_t::value_type JSON_MIME{"Content-type", "application/json;charset=utf-8"};
  const headers_t::value_type JS_MIME{"Content-type", "application/javascript;charset=utf-8"};
}

namespace valhalla {
namespace thor {

worker_t::result_t thor_worker_t::trace_route(
    const boost::property_tree::ptree &request,
    const std::string &request_str) {
  parse_shape(request);
  parse_costing(request);

  worker_t::result_t result{true};
  //get time for start of request
  auto s = std::chrono::system_clock::now();
  // Forward the original request
  result.messages.emplace_back(request_str);

  // call Meili for map matching to get a collection of pathLocation Edges
  // Create a matcher
  MapMatcher* matcher;
  try {
    matcher = matcher_factory_.Create(config);
  } catch (const std::invalid_argument& ex) {
    //return jsonify_error({400, 499}, request_info, std::string(ex.what()));
    throw std::runtime_error(std::string(ex.what()));
  }

  std::vector<Measurement> sequence;
  for (const auto& coord: shape) {
    sequence.emplace_back(coord, gps_accuracy, search_radius);
  }

  // Create the vector of matched path results
  std::vector<meili::MatchResult> results;
  for (size_t i = 0; i < sequence.size(); i++) {
    results = (matcher->OfflineMatch(sequence));
  }
  LOG_INFO("Matched Results from input trace: ");
  for(const auto& result : results) {
    LOG_INFO(std::to_string(result.lnglat().first) + ", " + std::to_string(result.lnglat().second));
  }

  thor::MapMatchingRoute mapmatching_route;
  //Post-process the Path Locations to construct a vector of PathInfo and send to TripPathBuilder
  std::vector<PathInfo> path_edges = mapmatching_route.FormPath(matcher, results, mode_costing, mode);

  // Set origin and destination from map matching results
  baldr::PathLocation origin = matcher->mapmatching().state(results.front().stateid()).candidate();
  baldr::PathLocation destination = matcher->mapmatching().state(results.back().stateid()).candidate();
  std::vector<baldr::PathLocation> through_loc;
  auto trip_path = thor::TripPathBuilder::Build(matcher->graphreader(), mode_costing,
                                                path_edges, origin,
                                                destination, through_loc);
  result.messages.emplace_back(trip_path.SerializeAsString());

  //get processing time for thor
  auto e = std::chrono::system_clock::now();
  std::chrono::duration<float, std::milli> elapsed_time = e - s;
  return result;
}

}
}
