#pragma once
#include "osr/routing/algorithms/a_star.h"
namespace osr {

template <typename Profile>
struct a_star_bi {
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;
  using node_h = typename a_star<Profile>::node_h;
  using cost_map = typename ankerl::unordered_dense::map<key, entry, hash>;

  struct get_bucket {
    cost_t operator()(node_h const& n) { return n.cost + n.heuristic; }
  };

  void add(label const l,
           ways const& w,
           location const& loc,
           cost_map& cost,
           dial<node_h, get_bucket>& d) {
    if (cost[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                            node::invalid())) {
      d.push(node_h{l, l.cost_, heuristic(l, w, loc)});
    }
  }

  void add_start(label const l, ways const& w) {
    add(l, w, end_loc_, cost1_, pq1_);
  }

  void add_end(label const l, ways const& w) {
    add(l, w, start_loc_, cost2_, pq2_);
  }

  void clear_mp() { meet_point = meet_point.invalid(); }

  void reset(cost_t max, location const& start_loc, location const& end_loc) {
    pq1_.clear();
    pq2_.clear();
    pq1_.n_buckets(max + 1U);
    pq2_.n_buckets(max + 1U);

    meet_point = meet_point.invalid();
    cost1_.clear();
    cost2_.clear();
    expanded_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;

    auto const start_coord = geo::latlng_to_merc(start_loc.pos_);
    auto const end_coord = geo::latlng_to_merc(end_loc.pos_);

    auto const dx = start_coord.x_ - end_coord.x_;
    auto const dy = start_coord.y_ - end_coord.y_;
    auto const dist = std::sqrt(dx * dx + dy * dy);

    PI = Profile::heuristic(dist) / 2;
  }

  cost_t heuristic(label const l, ways const& w, location const& loc) {
    auto const node_coord =
        geo::latlng_to_merc(w.get_node_pos(l.n_).as_latlng());
    auto const loc_coord = geo::latlng_to_merc(loc.pos_);

    auto const dx = node_coord.x_ - loc_coord.x_;
    auto const dy = node_coord.y_ - loc_coord.y_;

    auto const dist = std::sqrt(dx * dx + dy * dy);

    auto const other_loc = loc == start_loc_ ? end_loc_ : start_loc_;
    auto const other_coord = geo::latlng_to_merc(other_loc.pos_);

    auto const other_dx = node_coord.x_ - other_coord.x_;
    auto const other_dy = node_coord.y_ - other_coord.y_;

    auto const other_dist =
        std::sqrt(other_dx * other_dx + other_dy * other_dy);

    return Profile::heuristic(0.5 * (dist) - (other_dist)) +
           PI;
  }

  /*double newtonSqrt(double x) {
    double x1 = x;
    double x2 = x / 2;
    while (std::abs(x1 - x2) >= 0.0001) {
      x1 = x2;
      x2 = (x1 + x / x1) / 2;
    }
    return x2;
  }*/

  cost_t get_cost_from_start(node const n) const {
    auto const it = cost1_.find(n.get_key());
    return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
  }

  cost_t get_cost_from_end(node const n) const {
    auto const it = cost2_.find(n.get_key());
    return it != end(cost2_) ? it->second.cost(n) : kInfeasible;
  }

  cost_t get_cost_to_mp(node const n) const {
    auto const cost1 = get_cost_from_start(n);
    auto const cost2 = get_cost_from_end(n);
    if (cost1 == kInfeasible || cost2 == kInfeasible) {
      return kInfeasible;
    }
    return cost1 + cost2;
  }

  template <direction SearchDir, bool WithBlocked, typename fn>
  std::optional<node> run(ways const& w,
                          ways::routing const& r,
                          cost_t const max,
                          bitvec<node_idx_t> const* blocked,
                          dial<node_h, get_bucket>& d,
                          cost_map& cost_map,
                          fn get_cost,
                          location& loc) {
    auto curr_node_h = d.pop();

    auto l = curr_node_h.l;
    auto curr = l.get_node();

    if (get_cost(curr) < l.cost()) {
      return std::nullopt;
    }

    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t) {
          auto const total = l.cost() + cost;
          if (total < max &&
              cost_map[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node());
            node_h next_h = node_h{next, next.cost_, heuristic(next, w, loc)};
            if (next_h.cost + next_h.heuristic < max) {
              d.push(next_h);
            }
          }
        });
    return curr;
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked) {

    auto best_cost = kInfeasible - PI*2;
    auto next_item1 = pq1_.buckets_[pq1_.get_next_bucket()].back();
    auto next_item2 = pq2_.buckets_[pq2_.get_next_bucket()].back();

    auto top_f = next_item1.priority();
    auto top_r = next_item2.priority();

    while (
        !pq1_.empty() && !pq2_.empty() &&
        (top_f + top_r <
         best_cost +
             PI * 2 || best_cost == kInfeasible)) {

      auto curr1 = run<SearchDir, WithBlocked>(
          w, r, max, blocked, pq1_, cost1_,
          [this](auto curr) { return get_cost_from_start(curr); }, end_loc_);

      auto curr2 = run<opposite(SearchDir), WithBlocked>(
          w, r, max, blocked, pq2_, cost2_,
          [this](auto curr) { return get_cost_from_end(curr); }, start_loc_);

      if (curr1 != std::nullopt) {
        if (!expanded_.contains(curr1.value().n_)) {
          //std::cout << "pq1 expands " << static_cast<uint32_t>(curr1.value().n_) << std::endl;
          expanded_.emplace(curr1.value().n_);
        } else if (get_cost_to_mp(curr1.value()) < best_cost) {
          meet_point = curr1.value();
          //std::cout << "-----meet point has been found by pq1 " << static_cast<uint32_t>(meet_point.n_) << std::endl;
          best_cost = get_cost_to_mp(curr1.value());
          //std::cout << "best cost " << static_cast<uint32_t>(best_cost) << std::endl;
        }
      }
      if (curr2 != std::nullopt) {
        if (!expanded_.contains(curr2.value().n_)) {
          //std::cout << "pq2 expands " << static_cast<uint32_t>(curr2.value().n_) << std::endl;
          expanded_.emplace(curr2.value().n_);
        } else if (get_cost_to_mp(curr2.value()) < best_cost) {
          meet_point = curr2.value();
          //std::cout << "-----meet point has been found by pq2 " << static_cast<uint32_t>(meet_point.n_) << std::endl;
          best_cost = get_cost_to_mp(curr2.value());
          //std::cout << "best cost " << static_cast<uint32_t>(best_cost) << std::endl;
        }
      }

      top_f = pq1_.buckets_[pq1_.get_next_bucket()].back().priority();
      //std::cout << "top_f - " << static_cast<uint32_t>(top_f) << std::endl;
      top_r = pq2_.buckets_[pq2_.get_next_bucket()].back().priority();
      //std::cout << "top_r - " << static_cast<uint32_t>(top_r) << std::endl;
      //std::cout << "best cost " << static_cast<uint32_t>(best_cost) << std::endl;
    }
  }

  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           direction const dir) {
    if (blocked == nullptr) {
      dir == direction::kForward
          ? run<direction::kForward, false>(w, r, max, blocked)
          : run<direction::kBackward, false>(w, r, max, blocked);
    } else {
      dir == direction::kForward
          ? run<direction::kForward, true>(w, r, max, blocked)
          : run<direction::kBackward, true>(w, r, max, blocked);
    }
  }

  dial<node_h, get_bucket> pq1_{get_bucket{}};
  dial<node_h, get_bucket> pq2_{get_bucket{}};
  location start_loc_;
  location end_loc_;
  hash_set<node_idx_t> expanded_;
  node meet_point;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;
  cost_t PI;
};

}  // namespace osr