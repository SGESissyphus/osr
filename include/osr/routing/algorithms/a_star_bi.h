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

  void add_start(label const l, ways const& w) {
    if (cost1_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                              node::invalid())) {
      minHeap1_.push_back(node_h{l, 0, heuristic(l, w)});
    }
  }

  void add_end(label const l, ways const& w) {
    if (cost2_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                              node::invalid())) {
      minHeap2_.push_back(node_h{l, 0, heuristic_Start(l, w)});
    }
  }

  void reset(cost_t,
             location const& start_loc,
             location const& end_loc,
             way_candidate start,
             way_candidate end) {
    minHeap1_.clear();
    minHeap2_.clear();
    cost1_.clear();
    cost2_.clear();
    expanded_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    start_ = start;
    end_ = end;
  }

  // TODO Heuristics

  cost_t heuristic(label const l, ways const& w) {
    auto const node_latlng = w.get_node_pos(l.n_).as_latlng();
    return geo::distance(node_latlng, end_loc_.pos_);
  }

  cost_t heuristic_to_start(label const l, ways const& w) {
    auto const coord_node = w.get_node_pos(l.n_).as_latlng();
    return geo::distance(coord_node, start_loc_.pos_);
  }

  // TODO get_cost not modified for bi-directional
  cost_t get_cost_from_start(node const n) const {
    auto const it = cost1_.find(n.get_key());
    return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
  }

  cost_t get_cost_from_end(node const n) const {
    auto const it = cost2_.find(n.get_key());
    return it != end(cost2_) ? it->second.cost(n) : kInfeasible;
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked) {

    std::make_heap(minHeap1_.begin(), minHeap2_.end(), std::greater<node_h>{});
    std::make_heap(minHeap2_.begin(), minHeap2_.end(), std::greater<node_h>{});
    node curr1;
    node curr2;

    while (minHeap1_.size() > 0 && minHeap2_.size() > 0) {
      curr1 = run_start_to_end(w, r, max, blocked);
      curr2 = run_end_to_start(w, r, max, blocked);

      if (*curr1 != nullptr) {
        if (!expanded_.contains(curr1)) {
          expanded_.insert(curr1);
        } else {
          meet_point = curr1;
          break;
        }
      } else if (*curr2 != nullptr) {
        if (!expanded_.contains(curr2)) {
          expanded_.insert(curr2);
        } else {
          meet_point = curr2;
          break;
        }
      }
      if (curr1 == end_.right_.node_) {
        meet_point = curr1;
        return;
      } else if (curr2 == start_.left_.node_) {
        meet_point = curr2;
        return;
      }
    }
  }

  std::optional<node> run_start_to_end(ways const& w,
                                       ways::routing const& r,
                                       cost_t const max,
                                       bitvec<node_idx_t> const* blocked) {
    std::pop_heap(minHeap1_.begin(), minHeap1_.end(), std::greater<node_h>{});
    auto curr_node_h = minHeap1_.back();
    minHeap1_.pop_back();

    auto l = curr_node_h.l;
    auto curr = l.get_node();

    if (get_cost_from_start(curr) < l.cost()) {
      return std::nullopt;  // TODO check for good return value
    }
    if (get_cost_from_end(curr) < l.cost()) {
      return std::nullopt;  // TODO check for good return value
    }

    Profile::template adjacent<direction::kForward, false>(
        r, curr, blocked,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t) {
          auto const total = l.cost() + cost;
          if (total < max &&
              cost1_[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node());
            node_h next_h = node_h{next, next.cost_, heuristic(next, w)};
            minHeap1_.push_back(next_h);
            std::push_heap(minHeap1_.begin(), minHeap1_.end(),
                           std::greater<node_h>{});
          }
        });

    return curr;
  }

  std::optional<node> run_end_to_start(ways const& w,
                                       ways::routing const& r,
                                       cost_t const max,
                                       bitvec<node_idx_t> const* blocked) {
    std::pop_heap(minHeap2_.begin(), minHeap2_.end(), std::greater<node_h>{});
    auto curr_node_h = minHeap2_.back();
    minHeap2_.pop_back();

    auto l = curr_node_h.l;
    auto curr = l.get_node();

    if (get_cost_from_start(curr) < l.cost()) {
      return std::nullopt;  // TODO check for good return value
    }

    if (get_cost_from_end(curr) < l.cost()) {
      return std::nullopt;  // TODO check for good return value
    }

    Profile::template adjacent<direction::kBackward, false>(
        r, curr, blocked,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t) {
          auto const total = l.cost() + cost;
          if (total < max &&
              cost2_[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node());
            node_h next_h =
                node_h{next, next.cost_, heuristic_to_start(next, w)};
            minHeap2_.push_back(next_h);
            std::push_heap(minHeap2_.begin(), minHeap2_.end(),
                           std::greater<node_h>{});
          }
        });

    return curr;
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
  way_candidate start_;
  way_candidate end_;
  hash_set<node> expanded_;
  node meet_point;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;

};  // struct a_star_bi

}  // namespace osr