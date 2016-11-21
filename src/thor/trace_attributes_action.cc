#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/geojson.h>
#include <valhalla/baldr/errorcode_util.h>
#include <valhalla/baldr/json.h>

#include "thor/service.h"

using namespace valhalla;
using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::sif;
using namespace valhalla::thor;


namespace {
  const headers_t::value_type CORS { "Access-Control-Allow-Origin", "*" };
  const headers_t::value_type JSON_MIME { "Content-type", "application/json;charset=utf-8" };
  const headers_t::value_type JS_MIME { "Content-type", "application/javascript;charset=utf-8" };


  /*
   * Valhalla data revision / Id - OSM changeset number.
   * Units. (miles or meters).
   * Shape. This is the shape along the trace, snapped to the graph/route network. This is returned as a compressed string.
   * List of edges and intersections. Each edge describes a section of road or path between 2 intersections. Each intersection describes the intersection at the end of the corresponding edge.
   *
   */
  /*json::MapPtr serialize(const boost::optional<std::string>& id, const std::list<valhalla::odin::TripDirections>& legs, std::ostringstream& stream, worker_t::result_t& result) {
     json::ArrayPtr edges = json::array({});
     json::ArrayPtr intersections = json::array({});
     auto attributes = json::map({});
     auto edge = json::map({});

     auto json = json::map
     ({
       {"trip", json::map
         ({
           {"locations", locations(directions_legs)},
           {"summary", summary(directions_legs)},
           {"legs", legs(directions_legs)},
           {"status_message", string("Found route between points")}, //found route between points OR cannot find route between points
           {"status", static_cast<uint64_t>(0)}, //0 success
           {"units", std::string((directions_options.units() == valhalla::odin::DirectionsOptions::kKilometers) ? "kilometers" : "miles")},
           {"language", directions_options.language()}
         })
       }
     });

     edge->emplace("start_index", maneuver.sign().exit_branch_elements(i).text());
     edge->emplace("end_index", maneuver.sign().exit_branch_elements(i).text());
     edge->emplace("edge_id", maneuver.sign().exit_branch_elements(i).text());
     edge->emplace("way_id", maneuver.sign().exit_branch_elements(i).text());
     //..and so on...
     edges->emplace_back(edge);

     auto intersection = json::map({});
     intersection->emplace("type", maneuver.sign().exit_branch_elements(i).text());
     intersection->emplace("timezone", maneuver.sign().exit_branch_elements(i).text());
     //..and so on...
     intersections->emplace_back(intersection);

     auto json = json::map({
        {"units", units},
     });
     json->emplace("shape", json::array({locations(correlated_t)}));
     json->emplace("attributes",edges);
     json->emplace
     if (id)
       json->emplace("data_id", *id);
     return json;
   }*/
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
    const std::string &request_str, const bool header_dnt) {
  parse_locations(request);
  parse_shape(request);
  parse_costing(request);

  //get time for start of request
  auto s = std::chrono::system_clock::now();
  worker_t::result_t result { true };
  // Forward the original request
  result.messages.emplace_back(request_str);

  // If the exact points from a prior route that was run agains the Valhalla road network,
  //then we can traverse the exact shape to form a path by using edge-walking algorithm
  if (!route_match(result)) {
    //If no Valhalla route match, then use meili map matching
    //to match to local route network. No shortcuts are used and detailed
    //information at every intersection becomes available.
    map_match(result);
  }

  std::stringstream stream(result);
  boost::property_tree::ptree attributes;
  try{
    boost::property_tree::read_json(stream, attributes);
  } catch(...) {
    return jsonify_error({500, 200}, info, jsonp);
  }

  json::MapPtr json;
  auto id = attributes.get_optional<std::string>("id");


 /* for(const auto& directions_leg : directions_legs) {
    auto leg = json::map({});
    leg->emplace("shape", directions_leg.shape());
  }*/
  //serialize output to Thor
  //json = serialize(id, result);


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
  if (!header_dnt && (elapsed_time.count() / correlated.size()) > long_request) {
    LOG_WARN("thor::trace_attributes elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
    LOG_WARN("thor::trace_attributes exceeded threshold::"+ request_str);
    midgard::logging::Log("valhalla_thor_long_request_trace_attributes", " [ANALYTICS] ");
  }

  return result;
}
}
}
