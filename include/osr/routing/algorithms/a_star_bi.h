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
  using heap = std::vector<node_h>;

  void add(label const l, ways const& w, location const& loc, cost_map& cost, heap& heap) {
    if (cost[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                            node::invalid())) {
      heap.push_back(node_h{l, 0, heuristic(l, w, loc)});
    }
  }

  void add_start(label const l, ways const& w) {
    add(l, w, end_loc_, cost1_, minHeap1_);
  }

  void add_end(label const l, ways const& w) {
    add(l, w, start_loc_, cost2_, minHeap2_);
  }

  void clear_mp() { meet_point = meet_point.invalid(); }

  void reset(cost_t,
             location const& start_loc,
             location const& end_loc,
             match_t const& start,
             match_t const& end) {
    minHeap1_.clear();
    minHeap2_.clear();
    meet_point = meet_point.invalid();
    cost1_.clear();
    cost2_.clear();
    expanded_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    start_ = start;
    end_ = end;
  }

  cost_t heuristic(label const l, ways const& w, location const& loc) {
    auto const node_coord =
        geo::latlng_to_merc(w.get_node_pos(l.n_).as_latlng());
    auto const loc_coord = geo::latlng_to_merc(loc.pos_);

    auto const dx = node_coord.x_ - loc_coord.x_;
    auto const dy = node_coord.y_ - loc_coord.y_;

    auto const dist = newtonSqrt(dx * dx + dy * dy);

    return Profile::heuristic(dist);
  }

  double newtonSqrt(double x) {
    double x1 = x;
    double x2 = x / 2;
    while (std::abs(x1 - x2) >= 0.0001) {
      x1 = x2;
      x2 = (x1 + x / x1) / 2;
    }
    return x2;
  }

  cost_t get_cost_to_mp(node const n) const {
    auto const cost1 = get_cost_from_start(n);
    auto const cost2 = get_cost_from_end(n);
    if (cost1 == kInfeasible || cost2 == kInfeasible) {
      return kInfeasible;
    }
    return cost1 + cost2;
  }

  cost_t get_cost_from_start(node const n) const {
    auto const it = cost1_.find(n.get_key());
    return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
  }

  cost_t get_cost_from_end(node const n) const {
    auto const it = cost2_.find(n.get_key());
    return it != end(cost2_) ? it->second.cost(n) : kInfeasible;
  }

  template <direction SearchDir, bool WithBlocked, typename fn>
  std::optional<node> run(ways const& w,
                                  ways::routing const& r,
                                  cost_t const max,
                                  bitvec<node_idx_t> const* blocked,
                                  heap& heap,
                                  cost_map& cost_map,
                                  fn get_cost,
                                  location& loc) {
    std::pop_heap(heap.begin(), heap.end(), std::greater<node_h>{});
    auto curr_node_h = heap.back();
    heap.pop_back();

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
            heap.push_back(next_h);
            std::push_heap(heap.begin(), heap.end(), std::greater<node_h>{});
          }
        });
    return curr;
  }


  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked) {
    std::make_heap(minHeap1_.begin(), minHeap1_.end(), std::greater<node_h>{});
    std::make_heap(minHeap2_.begin(), minHeap2_.end(), std::greater<node_h>{});

    auto buffer = 750;
    bool found = false;
    auto best_cost = kInfeasible;

    while (!minHeap1_.empty() && !minHeap2_.empty() && buffer > 0) {
      auto curr1 = run<SearchDir, WithBlocked>(
          w, r, max, blocked, minHeap1_, cost1_,
          [this](auto curr) { return get_cost_from_start(curr); },
          end_loc_
      );
      auto curr2 = run<opposite(SearchDir), WithBlocked>(
          w, r, max, blocked, minHeap2_, cost2_,
          [this](auto curr) { return get_cost_from_end(curr); },
          start_loc_
      );

      if (curr1 != std::nullopt) {
        if (!expanded_.contains(curr1.value())) {
          expanded_.emplace(curr1.value());
        } else if (get_cost_to_mp(curr1.value()) < best_cost) {
          meet_point = curr1.value();
          best_cost = get_cost_to_mp(curr1.value());
          found = true;
        }
      }
      if (curr2 != std::nullopt) {
        if (!expanded_.contains(curr2.value())) {
          expanded_.emplace(curr2.value());
        } else if (get_cost_to_mp(curr2.value()) < best_cost) {
          meet_point = curr2.value();
          best_cost = get_cost_to_mp(curr2.value());
          found = true;
        }
      }
      if (found) {
        buffer--;
      }
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

  std::vector<node_h> minHeap1_;
  std::vector<node_h> minHeap2_;
  location start_loc_;
  location end_loc_;
  match_t start_;
  match_t end_;
  hash_set<node> expanded_;
  node meet_point;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;

};  // struct a_star_bi

}  // namespace osr