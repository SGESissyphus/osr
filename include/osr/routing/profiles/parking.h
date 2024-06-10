#pragma once

#include "osr/ways.h"

namespace osr {

// dürfen wir davon ausgehen, dass Autofahrer != Rollstuhlfahrer sind?
template <bool IsWheelchair>

// kUturnPenalty needs to be added to the foot struct
// rename to parking struct
struct parking {
  static constexpr auto const kUturnPenalty = cost_t{120U};
  static constexpr auto const kMaxMatchDistance = 150U;
  static constexpr auto const kOffroadPenalty = 3U;
  

  struct node {
    friend bool operator==(node, node) = default;

    static constexpr node invalid() noexcept {
      return {.n_ = node_idx_t::invalid(), .lvl_{level_t::invalid(), .way_ = 0U, .dir_ = direction::kForward}};
    }

    constexpr node_idx_t get_node() const noexcept { return n_; }
    constexpr node get_key() const noexcept { return *this; }

    std::ostream& print(std::ostream& out, ways const& w) const {
      return out << "(node=" << w.node_to_osm_[n_]
                 << ", level=" << to_float(lvl_)
                 << ", dir=" << to_str(dir_)
                 << ", way=" << w.way_osm_idx_[w.node_ways_[n_][way_]] << ")";
    }

    node_idx_t n_;
    level_t lvl_;
    way_pos_t way_;
    direction dir_;
    bool is_parked_;
  };

  // there are different keys used in foot and car
  using key = node;

  // what exactly is an entry?
  struct entry {
    constexpr std::optional<node> pred(node) const noexcept {
      // car uses an index to get the pred, foot uses the node directly
      return pred_ == node_idx_t::invalid()
                 ? std::nullopt
                 : std::optional{node{pred_, pred_lvl_}}; // pred_way_ and pred_dir_ are missing
    }
    // cost saved in node vs cost saved in entry
    constexpr cost_t cost(node) const noexcept { return cost_; }

    // car uses an index to get the cost and pred_way_ and pred_dir_ additionally
    constexpr bool update(node, cost_t const c, node const pred) noexcept {
      if (c < cost_) {
        cost_ = c;
        pred_ = pred.n_;
        pred_lvl_ = pred.lvl_;
        return true;
      }
      return false;
    }

    // get_node() is missing

    // get_index() is missing

    // to_dir() is missing

    // to_bool() is missing

    // node_idk_t not in form of array, array way_pos_t is missing, bitset kn missing, array cost_t is missing
    node_idx_t pred_{node_idx_t::invalid()};
    level_t pred_lvl_;
    cost_t cost_{kInfeasible};
  };
  // way_ and dir_ are missing
  struct label {
    label(node const n, cost_t const c) : n_{n.n_}, lvl_{n.lvl_}, cost_{c} {}

    constexpr node get_node() const noexcept { return {n_, lvl_}; }
    constexpr cost_t cost() const noexcept { return cost_; }

    node_idx_t n_;
    level_t lvl_;
    cost_t cost_;
  };

  struct hash {
    using is_avalanching = void;
    auto operator()(node const n) const noexcept -> std::uint64_t {
      using namespace ankerl::unordered_dense::detail;
      return wyhash::mix(
          wyhash::hash(static_cast<std::uint64_t>(to_idx(n.lvl_))),
          wyhash::hash(static_cast<std::uint64_t>(to_idx(n.n_))));  // car only needs to_idx(n) insted of to_idx(n.lvl_) and to_idx(n.n_)
    }
  };

  template <typename Fn>
  static void resolve(ways const& w,
                      way_idx_t const way,
                      node_idx_t const n,
                      level_t const lvl,
                      Fn&& f) {
    auto const p = w.way_properties_[way];
    if (lvl == level_t::invalid() ||
        (p.from_level() == lvl || p.to_level() == lvl ||
         can_use_elevator(w, n, lvl))) {
      f(node{n, lvl == level_t::invalid() ? p.from_level() : lvl});
    }
  }

// i dont get what is happening here
  template <typename Fn>
  static void resolve_all(ways const& w,
                          node_idx_t const n,
                          level_t const lvl,
                          Fn&& f) {
    auto const ways = w.node_ways_[n];
    auto levels = hash_set<level_t>{};
    for (auto i = way_pos_t{0U}; i != ways.size(); ++i) {
      // TODO what's with stairs? need to resolve to from_level or to_level?
      auto const p = w.way_properties_[w.node_ways_[n][i]];
      if (lvl == level_t::invalid()) {
        if (levels.emplace(p.from_level()).second) {
          f(node{n, p.from_level()});
        }
        if (levels.emplace(p.to_level()).second) {
          f(node{n, p.to_level()});
        }
      } else if ((p.from_level() == lvl || p.to_level() == lvl ||
                  can_use_elevator(w, n, lvl)) &&
                 levels.emplace(lvl).second) {
        f(node{n, lvl});
      }
    }
  }

  template <direction SearchDir, typename Fn>
  static void adjacent(ways const& w, node const n, Fn&& fn) {

    bool isparked = n.is_parked_;

    for (auto const [way, i] :
         utl::zip_unchecked(w.node_ways_[n.n_], w.node_in_way_idx_[n.n_])) {
      auto const expand = [&](direction const way_dir, std::uint16_t const from,
                              std::uint16_t const to) {
        auto const target_node = w.way_nodes_[way][to];
        auto const target_node_prop = w.node_properties_[target_node];
        if (isparked && node_cost_walk(target_node_prop) == kInfeasible) {
          return;
        }

        if(!isparked && node_cost_drive(target_node_prop) == kInfeasible){
          return;
        }

        auto const target_way_prop = w.way_properties_[way];
        if (way_cost(target_way_prop, way_dir, 0U) == kInfeasible) {
          return;
        }

        if(isparked && w.is_restricted<SearchDir>(n.n_, n.way_, way_pos_t{i})) {
          return;
        }

        auto const node_prop = w.node_properties_[n];
        if(!isparked){
          auto const is_u_turn = way_pos_t{i} == n.way_ && way_dir == opposite(n.dir_);
          auto const dist = w.way_node_dist_[way][std::min(from, to)];
          auto const target = node{target_node, w.get_way_pos(target_node, way), way_dir};
          auto const cost  = way_cost(target_way_prop, way_dir, dist) +
                             node_cost_drive(target_node_prop) +
                             (is_u_turn ? kUturnPenalty : 0U);
          if(w.node_properties_[target_node].is_parking_){
            auto const target_unparked = node{target_node, w.get_way_pos(target_node, way), n.dir, false}; //where lvl_?
            auto const target_parked = node{target_node, w.get_way_pos(target_node, way), n.dir, true};
            fn(target_unparked, cost, dist, way, from, to);
            fn(target_parked, cost, dist, way, from, to);
          }
          else{
            fn(target, cost, dist, way, from, to);
          }

        } else {
          if (can_use_elevator(w, target_node, n.lvl_)) {
            for_each_elevator_level(
                w, target_node, [&](level_t const target_lvl) {
                  auto const dist = w.way_node_dist_[way][std::min(from, to)];
                  auto const cost = way_cost(target_way_prop, way_dir, dist) +
                                    node_cost_walk(target_node_prop);
                  fn(node{target_node, target_lvl}, cost, dist, way, from, to);
                });
          } else {
            auto const target_lvl = get_target_level(w, n.n_, n.lvl_, way);
            if (!target_lvl.has_value()) {
              return;
            }

            auto const dist = w.way_node_dist_[way][std::min(from, to)];
            auto const cost = way_cost(target_way_prop, way_dir, dist) +
                              node_cost_walk(target_node_prop);
            fn(node{target_node, *target_lvl}, cost, dist, way, from, to);
          }
        }
      };

      //why OU?
      if (i != 0U) {
        expand(flip<SearchDir>(direction::kBackward), i, i - 1);
      }
      if (i != w.way_nodes_[way].size() - 1U) {
        expand(flip<SearchDir>(direction::kForward), i, i + 1);
      }
    }
  }

  // probably parking spots need to be added here to change the routing
  static bool is_reachable(ways const& w,
                           node const n,
                           way_idx_t const way,
                           direction const way_dir,
                           direction const search_dir) {
    auto const target_way_prop = w.way_properties_[way];

    bool isparked = n.is_parked_;

    if(!isparked){
      if (way_cost(
              target_way_prop,
              search_dir == direction::kForward ? way_dir : opposite(way_dir),
              0U) == kInfeasible) {
        return false;
      }

      if (!get_target_level(w, n.n_, n.lvl_, way).has_value()) {
        return false;
      }

      return true;
    }
    else {
      if(way_cost(target_way_prop, way_dir, 0U) == kInfeasible){
        return false;
      }
      if(w.is_restricted(n.n_, n.way_, w.get_way_pos(n.n_, way), search_dir)){
        return false;
      }
      return true;
    }
  }

  static std::optional<level_t> get_target_level(ways const& w,
                                                 node_idx_t const from_node,
                                                 level_t const from_level,
                                                 way_idx_t const to_way) {
    auto const way_prop = w.way_properties_[to_way];

    if (IsWheelchair && way_prop.is_steps()) {
      return std::nullopt;
    }

    if (way_prop.is_steps()) {
      if (way_prop.from_level() == from_level) {
        return way_prop.to_level();
      } else if (way_prop.to_level() == from_level) {
        return way_prop.from_level();
      } else {
        return std::nullopt;
      }
    } else if (can_use_elevator(w, to_way, from_level)) {
      return from_level;
    } else if (can_use_elevator(w, from_node, way_prop.from_level(),
                                from_level)) {
      return way_prop.from_level();
    } else if (way_prop.from_level() == from_level) {
      return from_level;
    } else {
      return std::nullopt;
    }
  }

  // needed??? why do we have it twice?
  static bool can_use_elevator(ways const& w,
                               way_idx_t const way,
                               level_t const a,
                               level_t const b = level_t::invalid()) {
    return w.way_properties_[way].is_elevator() &&
           can_use_elevator(w, w.way_nodes_[way][0], a, b);
  }

  template <typename Fn>
  static void for_each_elevator_level(ways const& w,
                                      node_idx_t const n,
                                      Fn&& f) {
    auto const p = w.node_properties_[n];
    if (p.is_multi_level()) {
      for_each_set_bit(get_elevator_multi_levels(w, n),
                       [&](auto&& l) { f(level_t{l}); });
    } else {
      f(p.from_level());
      f(p.to_level());
    }
  }

  // why are there to bools with same name?
  static bool can_use_elevator(ways const& w,
                               node_idx_t const n,
                               level_t const a,
                               level_t const b = level_t::invalid()) {
    auto const p = w.node_properties_[n];
    if (!p.is_elevator()) {
      return false;
    }

    if (p.is_multi_level()) {
      auto const levels = get_elevator_multi_levels(w, n);
      return has_bit_set(levels, to_idx(a)) &&
             (b == level_t::invalid() || has_bit_set(levels, to_idx(b)));
    } else {
      return (a == p.from_level() || a == p.to_level()) &&
             (b == level_t::invalid() || b == p.from_level() ||
              b == p.to_level());
    }
  }

  static level_bits_t get_elevator_multi_levels(ways const& w,
                                                node_idx_t const n) {
    auto const it = std::lower_bound(
        begin(w.multi_level_elevators_), end(w.multi_level_elevators_), n,
        [](auto&& x, auto&& y) { return x.first < y; });
    assert(it != end(w.multi_level_elevators_) && it->first == n);
    return it->second;
  }

  // different costs for car and foot, what happens by changing to foot after car?
  static constexpr cost_t way_cost(way_properties const e,
                                   direction,
                                   std::uint16_t const dist) {
    if (e.is_foot_accessible() && (!IsWheelchair || !e.is_steps())) {
      return static_cast<cost_t>(std::round(dist / 1.2F));
    } else {
      return kInfeasible;
    }
  }

  // combine with car node_cost
  static constexpr cost_t node_cost_walk(node_properties const n) {
    return n.is_walk_accessible() ? (n.is_elevator() ? 90U : 0U) : kInfeasible;
  }

  static constexpr cost_t node_cost_drive(node_properties const& n) {
    return n.is_car_accessible() ? 0U : kInfeasible;
  }
};

}  // namespace osr