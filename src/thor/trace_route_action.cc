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
       for (const auto& coord: coords) {
         sequence.emplace_back(coord, gps_accuracy, search_radius);
       }

       // Create the vector of matched path results
       std::vector<meili::MatchResult> results;
       for (size_t i = 0; i < sequence.size(); i++) {
         results = (matcher->OfflineMatch(sequence));
       }

      thor::MapMatchingRoute mapmatching_route;
      //Post-process the Path Locations to construct a vector of PathInfo and send to TripPathBuilder
      std::vector<PathInfo> path_edges = mapmatching_route.FormPath(matcher, results, mode_costing, mode);

      //TODO: trippathbuilder

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

    /*std::list<valhalla::odin::TripPath> thor_worker_t::path_from_trace(std::vector<thor::PathInfo>path_edges, const std::string &costing, const boost::optional<int> &date_time_type, const std::string &request_str) {
      //get time for start of request
      auto s = std::chrono::system_clock::now();
      bool prior_is_node = false;
      std::vector<baldr::PathLocation> through_loc;
      baldr::GraphId through_edge;
    //  std::vector<thor::PathInfo> path_edges;
      std::string origin_date_time, dest_date_time;

      std::list<valhalla::odin::TripPath> trippaths;
      baldr::PathLocation& last_break_origin = correlated[0];
      for(auto path_location = ++correlated.cbegin(); path_location != correlated.cend(); ++path_location) {
        auto origin = *std::prev(path_location);
        auto destination = *path_location;

        if (date_time_type && (*date_time_type == 0 || *date_time_type == 1) &&
            !dest_date_time.empty() && origin.stoptype_ == Location::StopType::BREAK)
          origin.date_time_ = dest_date_time;

        // Through edge is valid if last destination was "through"
        if (through_edge.Is_Valid()) {
          update_origin(origin, prior_is_node, through_edge);
        } else {
          last_break_origin = origin;
        }

        // Get the algorithm type for this location pair
        thor::PathAlgorithm* path_algorithm = get_path_algorithm(costing,
                             origin, destination);

        // Get best path
        if (path_edges.size() == 0) {
          get_path(path_algorithm, origin, destination, path_edges);
          if (path_edges.size() == 0) {
            throw valhalla_exception_t{400, 442};
          }

          if (date_time_type && *date_time_type == 0 && origin_date_time.empty() &&
              origin.stoptype_ == Location::StopType::BREAK)
            last_break_origin.date_time_ = origin.date_time_;
        } else {
          // Get the path in a temporary vector
          std::vector<thor::PathInfo> temp_path;
          get_path(path_algorithm, origin, destination, temp_path);
          if (temp_path.size() == 0) {
            throw valhalla_exception_t{400, 442};
          }

          if (date_time_type && *date_time_type == 0 && origin_date_time.empty() &&
              origin.stoptype_ == Location::StopType::BREAK)
            last_break_origin.date_time_ = origin.date_time_;

          // Append the temp_path edges to path_edges, adding the elapsed
          // time from the end of the current path. If continuing along the
          // same edge, remove the prior so we do not get a duplicate edge.
          uint32_t t = path_edges.back().elapsed_time;
          if (temp_path.front().edgeid == path_edges.back().edgeid) {
            path_edges.pop_back();
          }
          for (auto edge : temp_path) {
            edge.elapsed_time += t;
            path_edges.emplace_back(edge);
          }
        }

        // Build trip path for this leg and add to the result if this
        // location is a BREAK or if this is the last location
        if (destination.stoptype_ == Location::StopType::BREAK ||
            path_location == --correlated.cend()) {
            // Form output information based on path edges
            auto trip_path = thor::TripPathBuilder::Build(reader, path_edges,
                                                          last_break_origin, destination, through_loc);

            if (date_time_type) {
              origin_date_time = *last_break_origin.date_time_;
              dest_date_time = *destination.date_time_;
            }

            // The protobuf path
            trippaths.emplace_back(std::move(trip_path));

            // Clear path edges and set through edge to invalid
            path_edges.clear();
            through_edge = baldr::GraphId();
        } else {
            // This is a through location. Save last edge as the through_edge
            prior_is_node = false;
            for(const auto& e : origin.edges) {
              if(e.id == path_edges.back().edgeid) {
                prior_is_node = e.begin_node() || e.end_node();
                break;
              }
            }
            through_edge = path_edges.back().edgeid;

            // Add to list of through locations for this leg
            through_loc.emplace_back(destination);
        }

        // If we have another one coming we need to clear
        if (--correlated.cend() != path_location)
          path_algorithm->Clear();
      }

      return trippaths;
    }*/

  }
}
