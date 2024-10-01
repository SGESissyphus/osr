#include "osr/routing/route.h"

#include "boost/thread/tss.hpp"

#include "utl/concat.h"
#include "utl/to_vec.h"
#include "utl/verify.h"

#include "osr/routing/algorithms/a_star.h"
#include "osr/routing/algorithms/a_star_bi.h"
#include "osr/routing/algorithms/dijkstra.h"
#include "osr/routing/profiles/bike.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/profiles/car_parking.h"
#include "osr/routing/profiles/foot.h"

namespace osr {

search_profile to_profile(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("foot"): return search_profile::kFoot;
    case cista::hash("wheelchair"): return search_profile::kWheelchair;
    case cista::hash("bike"): return search_profile::kBike;
    case cista::hash("car"): return search_profile::kCar;
    case cista::hash("car_parking"): return search_profile::kCarParking;
    case cista::hash("car_parking_wheelchair"):
      return search_profile::kCarParkingWheelchair;
  }
  throw utl::fail("{} is not a valid profile", s);
}

routing_algorithm to_algorithm(std::string_view s) {
  switch (cista::hash(s)) {
    case cista::hash("dijkstra"): return routing_algorithm::kDijkstra;
    case cista::hash("a_star"): return routing_algorithm::kAStar;
    case cista::hash("a_star_bi"): return routing_algorithm::kAStarBi;
  }
  throw utl::fail("{} is not a valid algorithm", s);
}

std::string_view to_str(search_profile const p) {
  switch (p) {
    case search_profile::kFoot: return "foot";
    case search_profile::kWheelchair: return "wheelchair";
    case search_profile::kCar: return "car";
    case search_profile::kBike: return "bike";
    case search_profile::kCarParking: return "car_parking";
    case search_profile::kCarParkingWheelchair: return "car_parking_wheelchair";
  }
  throw utl::fail("{} is not a valid profile", static_cast<std::uint8_t>(p));
}

struct connecting_way {
  way_idx_t way_;
  std::uint16_t from_, to_;
  bool is_loop_;
  std::uint16_t distance_;
};

template <direction SearchDir, bool WithBlocked, typename Profile>
connecting_way find_connecting_way(ways const& w,
                                   ways::routing const& r,
                                   bitvec<node_idx_t> const* blocked,
                                   typename Profile::node const from,
                                   typename Profile::node const to,
                                   cost_t const expected_cost) {
  auto conn = std::optional<connecting_way>{};
  Profile::template adjacent<SearchDir, WithBlocked>(
      r, from, blocked,
      [&](typename Profile::node const target, std::uint32_t const cost,
          distance_t const dist, way_idx_t const way, std::uint16_t const a_idx,
          std::uint16_t const b_idx) {
        if (target == to && cost == expected_cost) {
          auto const is_loop = r.is_loop(way) &&
                               static_cast<unsigned>(std::abs(a_idx - b_idx)) ==
                                   r.way_nodes_[way].size() - 2U;
          conn = {way, a_idx, b_idx, is_loop, dist};
        }
      });
  utl::verify(conn.has_value(), "no connecting way node/{} -> node/{} found",
              w.node_to_osm_[from.get_node()], w.node_to_osm_[to.get_node()]);
  return *conn;
}

template <typename Profile>
connecting_way find_connecting_way(ways const& w,
                                   bitvec<node_idx_t> const* blocked,
                                   typename Profile::node const from,
                                   typename Profile::node const to,
                                   cost_t const expected_cost,
                                   direction const dir) {
  auto const call = [&]<bool WithBlocked>() {
    if (dir == direction::kForward) {
      return find_connecting_way<direction::kForward, WithBlocked, Profile>(
          w, *w.r_, blocked, from, to, expected_cost);
    } else {
      return find_connecting_way<direction::kBackward, WithBlocked, Profile>(
          w, *w.r_, blocked, from, to, expected_cost);
    }
  };

  if (blocked == nullptr) {
    return call.template operator()<false>();
  } else {
    return call.template operator()<true>();
  }
}

template <typename Profile>
double add_path(ways const& w,
                ways::routing const& r,
                bitvec<node_idx_t> const* blocked,
                typename Profile::node const from,
                typename Profile::node const to,
                cost_t const expected_cost,
                std::vector<path::segment>& path,
                direction const dir) {
  auto const& [way, from_idx, to_idx, is_loop, distance] =
      find_connecting_way<Profile>(w, blocked, from, to, expected_cost, dir);
  auto j = 0U;
  auto active = false;
  auto& segment = path.emplace_back();
  segment.way_ = way;
  segment.dist_ = distance;
  segment.cost_ = expected_cost;
  segment.from_level_ = r.way_properties_[way].from_level();
  segment.to_level_ = r.way_properties_[way].to_level();
  segment.from_ = r.way_nodes_[way][from_idx];
  segment.to_ = r.way_nodes_[way][to_idx];

  for (auto const [osm_idx, coord] :
       infinite(reverse(utl::zip(w.way_osm_nodes_[way], w.way_polylines_[way]),
                        (from_idx > to_idx) ^ is_loop),
                is_loop)) {
    utl::verify(j++ != 2 * w.way_polylines_[way].size() + 1U, "infinite loop");
    if (!active && w.node_to_osm_[r.way_nodes_[way][from_idx]] == osm_idx) {
      active = true;
    }
    if (active) {
      if (w.node_to_osm_[r.way_nodes_[way][from_idx]] == osm_idx) {
        // Again "from" node, then it's shorter to start from here.
        segment.polyline_.clear();
      }

      segment.polyline_.emplace_back(coord);
      if (w.node_to_osm_[r.way_nodes_[way][to_idx]] == osm_idx) {
        break;
      }
    }
  }
  return distance;
}

template <typename Profile>
path reconstruct_a_bi(ways const& w,
                      bitvec<node_idx_t> const* blocked,
                      a_star_bi<Profile> const& a,
                      way_candidate const& start,
                      way_candidate const& end,
                      cost_t const cost,
                      direction const dir) {
  auto forward_n = a.meet_point;
  //std::cout << "-----meet point " << static_cast<uint32_t>(a.meet_point.n_) << std::endl;
  auto forward_segments = std::vector<path::segment>{};
  auto forward_dist = 0.0;

  while (true) {
    //std::cout << "-----node to reconstruct in forward " << static_cast<uint32_t>(forward_n.n_) << std::endl;
    auto const& e = a.cost1_.at(forward_n.get_key());
    auto const pred = e.pred(forward_n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(forward_n) - a.get_cost_from_start(*pred));
      forward_dist += add_path<Profile>(w, *w.r_, blocked, *pred, forward_n,
                                        expected_cost, forward_segments, dir);
    } else {
      break;
    }
    forward_n = *pred;
  }

  auto const& start_node_candidate =
      forward_n.get_node() == start.left_.node_ ? start.left_ : start.right_;

  forward_segments.push_back(
      {.polyline_ = start_node_candidate.path_,
       .from_level_ = start_node_candidate.lvl_,
       .to_level_ = start_node_candidate.lvl_,
       .from_ = dir == direction::kBackward ? forward_n.get_node()
                                            : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? forward_n.get_node()
                                         : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = kInfeasible,
       .dist_ = 0});

  auto backward_segments = std::vector<path::segment>{};
  auto backward_n = a.meet_point;
  auto backward_dist = 0.0;

  while (true) {
    //std::cout << "-----node to reconstruct in backward " << static_cast<uint32_t>(backward_n.n_) << std::endl;
    auto const& e = a.cost2_.at(backward_n.get_key());

    auto const pred = e.pred(backward_n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(backward_n) - a.get_cost_from_end(*pred));
      backward_dist +=
          add_path<Profile>(w, *w.r_, blocked, *pred, backward_n, expected_cost,
                            backward_segments, opposite(dir));
    } else {
      break;
    }
    backward_n = *pred;
  }

  auto const& end_node_candidate =
      backward_n.get_node() == end.left_.node_ ? end.left_ : end.right_;

  backward_segments.push_back(
      {.polyline_ = end_node_candidate.path_,
       .from_level_ = end_node_candidate.lvl_,
       .to_level_ = end_node_candidate.lvl_,
       .from_ = dir == direction::kBackward ? forward_n.get_node()
                                            : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? forward_n.get_node()
                                         : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = kInfeasible,
       .dist_ = 0});

  std::reverse(forward_segments.begin(), forward_segments.end());
  forward_segments.insert(forward_segments.end(), backward_segments.begin(),
                          backward_segments.end());

  auto total_dist = start_node_candidate.dist_to_node_ + forward_dist +
                    backward_dist + end_node_candidate.dist_to_node_;

  auto p =
      path{.cost_ = cost, .dist_ = total_dist, .segments_ = forward_segments};

  a.cost2_.at(backward_n.get_key()).write(backward_n, p);

  return p;
}

template <typename Profile>
path reconstruct_a(ways const& w,
                   bitvec<node_idx_t> const* blocked,
                   a_star<Profile> const& a,
                   way_candidate const& start,
                   node_candidate const& dest,
                   typename Profile::node const dest_node,
                   cost_t const cost,
                   direction const dir) {
  auto n = dest_node;
  auto segments = std::vector<path::segment>{{.polyline_ = dest.path_,
                                              .from_level_ = dest.lvl_,
                                              .to_level_ = dest.lvl_,
                                              .from_ = node_idx_t::invalid(),
                                              .to_ = node_idx_t::invalid(),
                                              .way_ = way_idx_t::invalid()}};
  auto dist = 0.0;
  while (true) {
    auto const& e = a.cost_.at(n.get_key());
    auto const pred = e.pred(n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(n) - a.get_cost(*pred));
      dist += add_path<Profile>(w, *w.r_, blocked, *pred, n, expected_cost,
                                segments, dir);
    } else {
      break;
    }
    n = *pred;
  }

  auto const& start_node =
      n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  segments.push_back(
      {.polyline_ = start_node.path_,
       .from_level_ = start_node.lvl_,
       .to_level_ = start_node.lvl_,
       .from_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = kInfeasible,
       .dist_ = 0});
  std::reverse(begin(segments), end(segments));
  auto p = path{.cost_ = cost,
                .dist_ = start_node.dist_to_node_ + dist + dest.dist_to_node_,
                .segments_ = segments};
  a.cost_.at(dest_node.get_key()).write(dest_node, p);
  return p;
}

template <typename Profile>
path reconstruct(ways const& w,
                 bitvec<node_idx_t> const* blocked,
                 dijkstra<Profile> const& d,
                 way_candidate const& start,
                 node_candidate const& dest,
                 typename Profile::node const dest_node,
                 cost_t const cost,
                 direction const dir) {
  auto n = dest_node;
  auto segments = std::vector<path::segment>{{.polyline_ = dest.path_,
                                              .from_level_ = dest.lvl_,
                                              .to_level_ = dest.lvl_,
                                              .from_ = node_idx_t::invalid(),
                                              .to_ = node_idx_t::invalid(),
                                              .way_ = way_idx_t::invalid()}};
  auto dist = 0.0;
  while (true) {
    auto const& e = d.cost_.at(n.get_key());
    auto const pred = e.pred(n);
    if (pred.has_value()) {
      auto const expected_cost =
          static_cast<cost_t>(e.cost(n) - d.get_cost(*pred));
      dist += add_path<Profile>(w, *w.r_, blocked, *pred, n, expected_cost,
                                segments, dir);
    } else {
      break;
    }
    n = *pred;
  }

  auto const& start_node =
      n.get_node() == start.left_.node_ ? start.left_ : start.right_;
  segments.push_back(
      {.polyline_ = start_node.path_,
       .from_level_ = start_node.lvl_,
       .to_level_ = start_node.lvl_,
       .from_ =
           dir == direction::kBackward ? n.get_node() : node_idx_t::invalid(),
       .to_ = dir == direction::kForward ? n.get_node() : node_idx_t::invalid(),
       .way_ = way_idx_t::invalid(),
       .cost_ = kInfeasible,
       .dist_ = 0});
  std::reverse(begin(segments), end(segments));
  auto p = path{.cost_ = cost,
                .dist_ = start_node.dist_to_node_ + dist + dest.dist_to_node_,
                .segments_ = segments};
  d.cost_.at(dest_node.get_key()).write(dest_node, p);
  return p;
}

template <typename Profile>
std::optional<std::tuple<node_candidate const*,
                         way_candidate const*,
                         typename Profile::node,
                         path>>
best_candidate(ways const& w,
               a_star<Profile>& a,
               level_t const lvl,
               match_t const& m,
               cost_t const max,
               direction const dir) {
  auto const get_best = [&](way_candidate const& dest,
                            node_candidate const* x) {
    auto best_node = typename Profile::node{};
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    Profile::resolve_all(*w.r_, x->node_, lvl, [&](auto&& node) {
      if (!Profile::is_dest_reachable(*w.r_, node, dest.way_,
                                      flip(opposite(dir), x->way_dir_),
                                      opposite(dir))) {
        return;
      }

      auto const target_cost = a.get_cost(node);
      if (target_cost == kInfeasible) {
        return;
      }

      auto const total_cost = target_cost + x->cost_;
      if (total_cost < max && total_cost < best_cost.cost_) {
        best_node = node;
        best_cost.cost_ = static_cast<cost_t>(total_cost);
      }
    });
    return std::pair{best_node, best_cost};
  };

  for (auto const& dest : m) {
    auto best_node = typename Profile::node{};
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    auto best = static_cast<node_candidate const*>(nullptr);

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid() && x->cost_ < max) {
        auto const [x_node, x_cost] = get_best(dest, x);
        if (x_cost.cost_ < max && x_cost.cost_ < best_cost.cost_) {
          best = x;
          best_node = x_node;
          best_cost = x_cost;
        }
      }
    }

    if (best != nullptr) {
      return std::tuple{best, &dest, best_node, best_cost};
    }
  }
  return std::nullopt;
}

template <typename Profile>
std::optional<std::tuple<node_candidate const*,
                         way_candidate const*,
                         typename Profile::node,
                         path>>
best_candidate(ways const& w,
               dijkstra<Profile>& d,
               level_t const lvl,
               match_t const& m,
               cost_t const max,
               direction const dir) {
  auto const get_best = [&](way_candidate const& dest,
                            node_candidate const* x) {
    auto best_node = typename Profile::node{};
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    Profile::resolve_all(*w.r_, x->node_, lvl, [&](auto&& node) {
      if (!Profile::is_dest_reachable(*w.r_, node, dest.way_,
                                      flip(opposite(dir), x->way_dir_),
                                      opposite(dir))) {
        return;
      }

      auto const target_cost = d.get_cost(node);
      if (target_cost == kInfeasible) {
        return;
      }

      auto const total_cost = target_cost + x->cost_;
      if (total_cost < max && total_cost < best_cost.cost_) {
        best_node = node;
        best_cost.cost_ = static_cast<cost_t>(total_cost);
      }
    });
    return std::pair{best_node, best_cost};
  };

  for (auto const& dest : m) {
    auto best_node = typename Profile::node{};
    auto best_cost = path{.cost_ = std::numeric_limits<cost_t>::max()};
    auto best = static_cast<node_candidate const*>(nullptr);

    for (auto const x : {&dest.left_, &dest.right_}) {
      if (x->valid() && x->cost_ < max) {
        auto const [x_node, x_cost] = get_best(dest, x);
        if (x_cost.cost_ < max && x_cost.cost_ < best_cost.cost_) {
          best = x;
          best_node = x_node;
          best_cost = x_cost;
        }
      }
    }

    if (best != nullptr) {
      return std::tuple{best, &dest, best_node, best_cost};
    }
  }
  return std::nullopt;
}

std::optional<path> try_direct(osr::location const& from,
                               osr::location const& to) {
  auto const dist = geo::distance(from.pos_, to.pos_);
  return dist < 5.0
             ? std::optional{path{.cost_ = 60U,
                                  .dist_ = dist,
                                  .segments_ = {path::segment{
                                      .polyline_ = {from.pos_, to.pos_},
                                      .from_level_ = from.lvl_,
                                      .to_level_ = to.lvl_,
                                      .from_ = node_idx_t::invalid(),
                                      .to_ = node_idx_t::invalid(),
                                      .way_ = way_idx_t::invalid(),
                                      .cost_ = 60U,
                                      .dist_ = static_cast<distance_t>(dist)}},
                                  .uses_elevator_ = false}}
             : std::nullopt;
}

void sort_way_candidates(std::vector<way_candidate>& to_match) {
  std::sort(to_match.begin(), to_match.end(),
            [](const way_candidate& a, const way_candidate& b) {
              return a.dist_to_way_ < b.dist_to_way_;
            });
}

template <typename Profile>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          a_star_bi<Profile>& a,
                          location const& from,
                          location const& to,
                          cost_t const max,
                          direction const dir,
                          double const max_match_distance,
                          bitvec<node_idx_t> const* blocked) {
  auto from_match =
      l.match<Profile>(from, false, dir, max_match_distance, blocked);
  auto to_match = l.match<Profile>(to, true, dir, max_match_distance, blocked);

  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  a.reset(max, from, to);

  for (auto const& start : from_match) {
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve_start_node(*w.r_, start.way_, nc->node_, from.lvl_,
                                    dir, [&](auto const node) {
                                      a.add_start({node, nc->cost_}, w);
                                    });
      }
    }
    if (a.pq1_.empty()) {
      continue;
    }
    for (auto const& end : to_match) {
      for (auto const* nc : {&end.left_, &end.right_}) {
        if (nc->valid() && nc->cost_ < max) {
          Profile::resolve_start_node(*w.r_, end.way_, nc->node_, to.lvl_,
                                      opposite(dir), [&](auto const node) {
                                        a.add_end({node, nc->cost_}, w);
                                      });
        }
      }
      if (a.pq2_.empty()) {
        continue;
      }
      a.clear_mp();
      a.run(w, *w.r_, max, blocked, dir);
      cost_t cost = 0U;
      if (a.meet_point.get_node() == node_idx_t::invalid() ||
          static_cast<uint32_t>(a.meet_point.get_node()) == 0) {
        continue;
      }

      if (a.cost1_.find(a.meet_point.get_key()) != a.cost1_.end()) {
        cost += a.cost1_.at(a.meet_point.get_key()).cost(a.meet_point);
      }
      if (a.cost2_.find(a.meet_point.get_key()) != a.cost2_.end()) {
        cost += a.cost2_.at(a.meet_point.get_key()).cost(a.meet_point);
      }

      return reconstruct_a_bi(w, blocked, a, start, end, cost, dir);
    }
  }
  return std::nullopt;
}

template <typename Profile>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          a_star<Profile>& a,
                          location const& from,
                          location const& to,
                          cost_t const max,
                          direction const dir,
                          double const max_match_distance,
                          bitvec<node_idx_t> const* blocked) {
  auto const from_match =
      l.match<Profile>(from, false, dir, max_match_distance, blocked);
  auto const to_match =
      l.match<Profile>(to, true, dir, max_match_distance, blocked);

  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  a.reset(max, to, to_match);

  for (auto const& start : from_match) {
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve_start_node(*w.r_, start.way_, nc->node_, from.lvl_,
                                    dir, [&](auto const node) {
                                      a.add_start({node, nc->cost_}, w);
                                    });
      }
    }

    a.run(w, *w.r_, max, blocked, dir);

    auto const c = best_candidate(w, a, to.lvl_, to_match, max, dir);

    if (c.has_value()) {
      auto const [nc, wc, node, p] = *c;
      return reconstruct_a<Profile>(w, blocked, a, start, *nc, node, p.cost_,
                                    dir);
    }
  }
  return std::nullopt;
}

template <typename Profile>
std::optional<path> route(ways const& w,
                          lookup const& l,
                          dijkstra<Profile>& d,
                          location const& from,
                          location const& to,
                          cost_t const max,
                          direction const dir,
                          double const max_match_distance,
                          bitvec<node_idx_t> const* blocked) {
  auto const from_match =
      l.match<Profile>(from, false, dir, max_match_distance, blocked);
  auto const to_match =
      l.match<Profile>(to, true, dir, max_match_distance, blocked);

  if (from_match.empty() || to_match.empty()) {
    return std::nullopt;
  }

  if (auto const direct = try_direct(from, to); direct.has_value()) {
    return *direct;
  }

  d.reset(max);

  for (auto const& start : from_match) {
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve_start_node(*w.r_, start.way_, nc->node_, from.lvl_,
                                    dir, [&](auto const node) {
                                      d.add_start({node, nc->cost_});
                                    });
      }
    }

    if (d.pq_.empty()) {
      continue;
    }

    d.run(*w.r_, max, blocked, dir);

    auto const c = best_candidate(w, d, to.lvl_, to_match, max, dir);
    if (c.has_value()) {
      auto const [nc, wc, node, p] = *c;
      return reconstruct<Profile>(w, blocked, d, start, *nc, node, p.cost_,
                                  dir);
    }
  }

  return std::nullopt;
}

/*template <typename Profile>
std::vector<std::optional<path>> route(
    ways const& w,
    lookup const& l,
    a_star<Profile>& a,
    location const& from,
    std::vector<location> const& to,
    cost_t const max,
    direction const dir,
    double const max_match_distance,
    bitvec<node_idx_t> const* blocked,
    std::function<bool(path const&)> const& do_reconstruct) {
  auto const from_match =
      l.match<Profile>(from, false, dir, max_match_distance, blocked);
  auto const to_match = utl::to_vec(to, [&](auto&& x) {
    return l.match<Profile>(x, true, dir, max_match_distance, blocked);
  });

  auto result = std::vector<std::optional<path>>{};
  result.resize(to.size());

  if (from_match.empty()) {
    return result;
  }

  a.reset(max, to, to_match);
  for (auto const& mp : from_match) {
    for (auto const* nc : {&mp.left_, &mp.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve_start_node(*w.r_, mp.way_, nc->node_, from.lvl_,
                                    dir, [&](auto const node) {
                                      a.add_start({node, nc->cost_});
                                    });
      }
    }

    a.run(w, *w.r_, max, blocked, dir);

    auto found = 0U;
    for (auto const [m, t, r] : utl::zip(to_match, to, result)) {
      if (r.has_value()) {
        ++found;
      } else if (auto const direct = try_direct(from, t);
direct.has_value()) { r = direct; } else { auto const c = best_candidate(w,
a, t.lvl_, m, max, dir); if (c.has_value()) { auto [nc, wc, n, p] = *c; if
(do_reconstruct(p)) { p = reconstruct<Profile>(w, blocked, a, mp, *nc, n,
p.cost_, dir);
          }
          r = std::make_optional(p);
          ++found;
        }
      }
    }

    if (found == result.size()) {
      return result;
    }
  }

  return result;
}
*/

template <typename Profile>
std::vector<std::optional<path>> route(
    ways const& w,
    lookup const& l,
    dijkstra<Profile>& d,
    location const& from,
    std::vector<location> const& to,
    cost_t const max,
    direction const dir,
    double const max_match_distance,
    bitvec<node_idx_t> const* blocked,
    std::function<bool(path const&)> const& do_reconstruct) {
  auto const from_match =
      l.match<Profile>(from, false, dir, max_match_distance, blocked);
  auto const to_match = utl::to_vec(to, [&](auto&& x) {
    return l.match<Profile>(x, true, dir, max_match_distance, blocked);
  });

  auto result = std::vector<std::optional<path>>{};
  result.resize(to.size());

  if (from_match.empty()) {
    return result;
  }

  d.reset(max);
  for (auto const& start : from_match) {
    for (auto const* nc : {&start.left_, &start.right_}) {
      if (nc->valid() && nc->cost_ < max) {
        Profile::resolve_start_node(
            *w.r_, start.way_, nc->node_, from.lvl_, dir, [&](auto const node) {
              auto label = typename Profile::label{node, nc->cost_};
              label.track(label, *w.r_, start.way_, node.get_node());
              d.add_start(label);
            });
      }
    }

    d.run(*w.r_, max, blocked, dir);

    auto found = 0U;
    for (auto const [m, t, r] : utl::zip(to_match, to, result)) {
      if (r.has_value()) {
        ++found;
      } else if (auto const direct = try_direct(from, t); direct.has_value()) {
        r = direct;
      } else {
        auto const c = best_candidate(w, d, t.lvl_, m, max, dir);
        if (c.has_value()) {
          auto [nc, wc, n, p] = *c;
          d.cost_.at(n.get_key()).write(n, p);
          if (do_reconstruct(p)) {
            p = reconstruct<Profile>(w, blocked, d, start, *nc, n, p.cost_,
                                     dir);
            p.uses_elevator_ = true;
          }
          r = std::make_optional(p);
          ++found;
        }
      }
    }

    if (found == result.size()) {
      return result;
    }
  }

  return result;
}

template <typename Profile>
dijkstra<Profile>& get_dijkstra() {
  static auto s = boost::thread_specific_ptr<dijkstra<Profile>>{};
  if (s.get() == nullptr) {
    s.reset(new dijkstra<Profile>{});
  }
  return *s.get();
}

template <typename Profile>
a_star<Profile>& get_a_star() {
  static auto s = boost::thread_specific_ptr<a_star<Profile>>{};
  if (s.get() == nullptr) {
    s.reset(new a_star<Profile>{});
  }
  return *s.get();
}

template <typename Profile>
a_star_bi<Profile>& get_a_star_bi() {
  static auto s = boost::thread_specific_ptr<a_star_bi<Profile>>{};
  if (s.get() == nullptr) {
    s.reset(new a_star_bi<Profile>{});
  }
  return *s.get();
}

std::vector<std::optional<path>> route(
    ways const& w,
    lookup const& l,
    search_profile const profile,
    location const& from,
    std::vector<location> const& to,
    cost_t const max,
    direction const dir,
    double const max_match_distance,
    bitvec<node_idx_t> const* blocked,
    std::function<bool(path const&)> const& do_reconstruct) {
  switch (profile) {
    case search_profile::kFoot:
      return route(w, l, get_dijkstra<foot<false, elevator_tracking>>(), from,
                   to, max, dir, max_match_distance, blocked, do_reconstruct);
    case search_profile::kWheelchair:
      return route(w, l, get_dijkstra<foot<true, elevator_tracking>>(), from,
                   to, max, dir, max_match_distance, blocked, do_reconstruct);
    case search_profile::kBike:
      return route(w, l, get_dijkstra<bike>(), from, to, max, dir,
                   max_match_distance, blocked, do_reconstruct);
    case search_profile::kCar:
      return route(w, l, get_dijkstra<car>(), from, to, max, dir,
                   max_match_distance, blocked, do_reconstruct);
    case search_profile::kCarParking:
      return route(w, l, get_dijkstra<car_parking<false>>(), from, to, max, dir,
                   max_match_distance, blocked, do_reconstruct);
    case search_profile::kCarParkingWheelchair:
      return route(w, l, get_dijkstra<car_parking<true>>(), from, to, max, dir,
                   max_match_distance, blocked, do_reconstruct);
  }
  throw utl::fail("not implemented");
}

std::optional<path> route(ways const& w,
                          lookup const& l,
                          search_profile const profile,
                          routing_algorithm const algo,
                          location const& from,
                          location const& to,
                          cost_t const max,
                          direction const dir,
                          double const max_match_distance,
                          bitvec<node_idx_t> const* blocked) {
  switch (algo) {
    case routing_algorithm::kDijkstra:
      return route_dijkstra(w, l, profile, from, to, max, dir,
                            max_match_distance, blocked);
    case routing_algorithm::kAStar:
      return route_a_star(w, l, profile, from, to, max, dir, max_match_distance,
                          blocked);
    case routing_algorithm::kAStarBi:
      return route_a_star_bi(w, l, profile, from, to, max, dir,
                             max_match_distance, blocked);
  }
  throw utl::fail("not implemented");
}

std::optional<path> route_a_star_bi(ways const& w,
                                    lookup const& l,
                                    search_profile const profile,
                                    location const& from,
                                    location const& to,
                                    cost_t const max,
                                    direction const dir,
                                    double const max_match_distance,
                                    bitvec<node_idx_t> const* blocked) {

  switch (profile) {
    case search_profile::kFoot:
      return route(w, l, get_a_star_bi<foot<false, elevator_tracking>>(), from,
                   to, max, dir, max_match_distance, blocked);
    case search_profile::kWheelchair:
      return route(w, l, get_a_star_bi<foot<true, elevator_tracking>>(), from,
                   to, max, dir, max_match_distance, blocked);
    case search_profile::kBike:
      return route(w, l, get_a_star_bi<bike>(), from, to, max, dir,
                   max_match_distance, blocked);
    case search_profile::kCar:
      return route(w, l, get_a_star_bi<car>(), from, to, max, dir,
                   max_match_distance, blocked);
    case search_profile::kCarParking:
      return route(w, l, get_a_star_bi<car_parking<false>>(), from, to, max,
                   dir, max_match_distance, blocked);
    case search_profile::kCarParkingWheelchair:
      return route(w, l, get_a_star_bi<car_parking<true>>(), from, to, max, dir,
                   max_match_distance, blocked);
  }
  throw utl::fail("not implemented");
}

std::optional<path> route_a_star(ways const& w,
                                 lookup const& l,
                                 search_profile const profile,
                                 location const& from,
                                 location const& to,
                                 cost_t const max,
                                 direction const dir,
                                 double const max_match_distance,
                                 bitvec<node_idx_t> const* blocked) {
  switch (profile) {
    case search_profile::kFoot:
      return route(w, l, get_a_star<foot<false, elevator_tracking>>(), from, to,
                   max, dir, max_match_distance, blocked);
    case search_profile::kWheelchair:
      return route(w, l, get_a_star<foot<true, elevator_tracking>>(), from, to,
                   max, dir, max_match_distance, blocked);
    case search_profile::kBike:
      return route(w, l, get_a_star<bike>(), from, to, max, dir,
                   max_match_distance, blocked);
    case search_profile::kCar:
      return route(w, l, get_a_star<car>(), from, to, max, dir,
                   max_match_distance, blocked);
    case search_profile::kCarParking:
      return route(w, l, get_a_star<car_parking<false>>(), from, to, max, dir,
                   max_match_distance, blocked);
    case search_profile::kCarParkingWheelchair:
      return route(w, l, get_a_star<car_parking<true>>(), from, to, max, dir,
                   max_match_distance, blocked);
  }
  throw utl::fail("not implemented");
}

std::optional<path> route_dijkstra(ways const& w,
                                   lookup const& l,
                                   search_profile const profile,
                                   location const& from,
                                   location const& to,
                                   cost_t const max,
                                   direction const dir,
                                   double const max_match_distance,
                                   bitvec<node_idx_t> const* blocked) {
  switch (profile) {
    case search_profile::kFoot:
      return route(w, l, get_dijkstra<foot<false, elevator_tracking>>(), from,
                   to, max, dir, max_match_distance, blocked);
    case search_profile::kWheelchair:
      return route(w, l, get_dijkstra<foot<true, elevator_tracking>>(), from,
                   to, max, dir, max_match_distance, blocked);
    case search_profile::kBike:
      return route(w, l, get_dijkstra<bike>(), from, to, max, dir,
                   max_match_distance, blocked);
    case search_profile::kCar:
      return route(w, l, get_dijkstra<car>(), from, to, max, dir,
                   max_match_distance, blocked);
    case search_profile::kCarParking:
      return route(w, l, get_dijkstra<car_parking<false>>(), from, to, max, dir,
                   max_match_distance, blocked);
    case search_profile::kCarParkingWheelchair:
      return route(w, l, get_dijkstra<car_parking<true>>(), from, to, max, dir,
                   max_match_distance, blocked);
  }
  throw utl::fail("not implemented");
}

template a_star<foot<true, osr::noop_tracking>>&
get_a_star<foot<true, osr::noop_tracking>>();

template a_star<foot<false, osr::noop_tracking>>&
get_a_star<foot<false, osr::noop_tracking>>();

template a_star_bi<foot<true, osr::noop_tracking>>&
get_a_star_bi<foot<true, osr::noop_tracking>>();

template a_star_bi<foot<false, osr::noop_tracking>>&
get_a_star_bi<foot<false, osr::noop_tracking>>();

template dijkstra<foot<true, osr::noop_tracking>>&
get_dijkstra<foot<true, osr::noop_tracking>>();

template dijkstra<foot<false, osr::noop_tracking>>&
get_dijkstra<foot<false, osr::noop_tracking>>();

}  // namespace osr