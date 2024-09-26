#pragma once

#include <string_view>
#include <vector>

#include "boost/json/object.hpp"

#include "osr/lookup.h"
#include "osr/routing/algorithms/a_star.h"
#include "osr/routing/algorithms/a_star_bi.h"
#include "osr/routing/algorithms/dijkstra.h"
#include "osr/types.h"
#include "osr/util/infinite.h"
#include "osr/util/reverse.h"

namespace osr {

enum class search_profile : std::uint8_t {
  kFoot,
  kWheelchair,
  kBike,
  kCar,
  kCarParking,
  kCarParkingWheelchair
};

void sort_way_candidates(std::vector<way_candidate>& to_match);

enum class routing_algorithm : std::uint8_t { kDijkstra, kAStar, kAStarBi };

search_profile to_profile(std::string_view);

routing_algorithm to_algorithm(std::string_view);

std::string_view to_str(search_profile);

struct path {
  struct segment {
    std::vector<geo::latlng> polyline_;
    level_t from_level_;
    level_t to_level_;
    node_idx_t from_, to_;
    way_idx_t way_;
    cost_t cost_{kInfeasible};
    distance_t dist_{0};
  };

  cost_t cost_{kInfeasible};
  double dist_{0.0};
  std::vector<segment> segments_{};
  bool uses_elevator_{false};
};

template <typename Profile>
dijkstra<Profile>& get_dijkstra();

template <typename Profile>
a_star<Profile>& get_a_star();

template <typename Profile>
a_star_bi<Profile>& get_a_star_bi();

std::vector<std::optional<path>> route(
    ways const&,
    lookup const&,
    search_profile,
    routing_algorithm,
    location const& from,
    std::vector<location> const& to,
    cost_t max,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr,
    std::function<bool(path const&)> const& = [](path const&) {
      return false;
    });

template <typename Profile>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          a_star<Profile>& a,
                          location const& from,
                          location const& to,
                          cost_t const max,
                          direction const dir,
                          double const max_match_distance,
                          bitvec<node_idx_t> const* blocked);
template <typename Profile>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          dijkstra<Profile>& d,
                          location const& from,
                          location const& to,
                          cost_t const max,
                          direction const dir,
                          double const max_match_distance,
                          bitvec<node_idx_t> const* blocked);

std::optional<path> route(ways const&,
                          lookup const&,
                          search_profile,
                          routing_algorithm,
                          location const& from,
                          location const& to,
                          cost_t max,
                          direction,
                          double max_match_distance,
                          bitvec<node_idx_t> const* blocked = nullptr);

std::optional<path> route_dijkstra(ways const&,
                                   lookup const&,
                                   search_profile,
                                   location const& from,
                                   location const& to,
                                   cost_t max,
                                   direction,
                                   double max_match_distance,
                                   bitvec<node_idx_t> const* blocked = nullptr);

std::optional<path> route_a_star(ways const&,
                                 lookup const&,
                                 search_profile,
                                 location const& from,
                                 location const& to,
                                 cost_t max,
                                 direction,
                                 double max_match_distance,
                                 bitvec<node_idx_t> const* blocked = nullptr);

std::optional<path> route_a_star_bi(
    ways const&,
    lookup const&,
    search_profile,
    location const& from,
    location const& to,
    cost_t max,
    direction,
    double max_match_distance,
    bitvec<node_idx_t> const* blocked = nullptr);
}  // namespace osr