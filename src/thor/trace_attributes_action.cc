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
          if (node.has_edge()) {
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
 // worker_t::result_t result { false  };
  // Forward the original request
 // result.messages.emplace_back(request_str);

  // If the exact points from a prior route that was run agains the Valhalla road network,
  //then we can traverse the exact shape to form a path by using edge-walking algorithm
  auto trip_path = route_match();
  if (trip_path.node().size() == 0) {
    //If no Valhalla route match, then use meili map matching
    //to match to local route network. No shortcuts are used and detailed
    //information at every intersection becomes available.
    trip_path = map_match();
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
  worker_t::result_t result{false};
  result.messages.emplace_back(response.to_string());
  return result;
}
}
}
