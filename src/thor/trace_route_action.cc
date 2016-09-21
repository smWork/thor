#include <prime_server/prime_server.hpp>

using namespace prime_server;

#include <valhalla/midgard/logging.h>
#include <valhalla/baldr/geojson.h>
#include <valhalla/meili/map_matcher_factory.h>
#include <valhalla/meili/map_matcher.h>


#include "thor/service.h"
#include "thor/mapmatching_route.h"

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

    worker_t::result_t  thor_worker_t::trace_route(const boost::property_tree::ptree &request, prime_server::http_request_t::info_t& request_info) {
      parse_shape(request);
      parse_costing(request);

      worker_t::result_t result{true};
      //get time for start of request
      auto s = std::chrono::system_clock::now();

      //TODO: call Meili for map matching to get a collection of pathLocation Edges
      //TODO: convert shape to vector of Measurements (const midgard::PointLL& lnglat, float gps_accuracy, float search_radius)

      // Create a matcher
       MapMatcher* matcher;
       try {
         matcher = matcher_factory_.Create(config);
       } catch (const std::invalid_argument& ex) {
         return jsonify_error({400, 499}, request_info, std::string(ex.what()));
        }

      std::vector<Measurement> sequence;
      for (const auto& coord: coords) {
        //TODO: need to change from hard-coding gps accuracy & search radius
        sequence.emplace_back(coord, gps_accuracy, search_radius);
      }

      // Create the vector of matched path results
      std::vector<MatchResult> matched_path;
      for (size_t i = 0; i < sequence.size(); i++) {
        matched_path = (matcher->OfflineMatch(sequence));
      }

      thor::MapMatchingRoute mapmatching;
      //TODO: Post-process the Path Locations to construct a vector of PathInfo and send to TripPathBuilder
      std::vector<PathInfo> pathInfoVector = mapmatching.FormPath(matched_path, mode_costing, reader, mode);

      //TODO: get directions from Odin
      //TODO: serialize in Tyr

      //turn it into json
      auto json = baldr::json::map({});
      auto id = request.get_optional<std::string>("id");
      if(id)
        json->emplace("id", *id);
      std::stringstream stream; stream << *json;

      //get processing time for thor
      auto e = std::chrono::system_clock::now();
      std::chrono::duration<float, std::milli> elapsed_time = e - s;
      //log request if greater than X (ms)
      /*if (!header_dnt && (elapsed_time.count() / correlated.size()) > long_request) {
       LOG_WARN("thor::trace_route_path elapsed time (ms)::"+ std::to_string(elapsed_time.count()));
       LOG_WARN("thor::trace_route exceeded threshold::"+ request_str);
       midgard::logging::Log("valhalla_thor_long_trace_route", " [ANALYTICS] ");
      }*/
      return result;
    }

  }
}
