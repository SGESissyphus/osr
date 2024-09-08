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

  void add_start(label const start, label const end) {
    if (cost1_[start.get_node().get_key()].update(start, start.get_node(), start.cost(),
                                                  node::invalid())) {
      minHeap1_.push_back(node_h{start, 0, 0});
    }
    if (cost2_[end.get_node().get_key()].update(end, end.get_node(), end.cost(),
                                                node::invalid())) {
      minHeap2_.push_back(node_h{end, 0, 0});
    }
  }

  void reset(cost_t, location const& start_loc, location const& end_loc) {
    minHeap1_.clear();
    minHeap2_.clear();
    cost1_.clear();
    cost2_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
  }

  cost_t heuristic(bool const first_turn, label const l, ways const& w) {
    auto const coord_node = w.get_node_pos(l.n_).as_latlng();
    auto const coord_end = first_turn ? end_loc_.pos_ : start_loc_.pos_;
    return geo::distance(coord_node, coord_end);;
  }

  template <direction SearchDir, bool WithBlocked>
  bool run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           ankerl::unordered_dense::map<key, entry, hash> cost,
           ankerl::unordered_dense::map<key, entry, hash> other_cost,
           std::vector<node_h>& heap){

    std::make_heap(heap.begin(), heap.end(), std::greater<node_h>{});
    std::pop_heap(heap.begin(), heap.end(), std::greater<node_h>{});
    auto curr_node_h = heap.back();
    auto l = curr_node_h.l;
    heap.pop_back();

    if(other_cost.contains(l.n_))
      return true;

    if (get_cost(l.get_node()) < l.cost()) {
      return false;
    }
    auto const curr = l.get_node();

    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t) {
          auto const total = l.cost() + cost;
          if (total < max &&
              cost[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node());
            node_h next_h = node_h{next, next.cost_, heuristic(next, w)};
            heap.push_back(next_h);
            std::push_heap(heap.begin(), heap.end(),
                           std::greater<node_h>{});
          }
        });
  }
  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked) {
    bool firsts_turn = true;
    while (!minHeap1_.empty() && !minHeap2_.empty()) {
      auto cost = cost1_;
      auto other_cost = cost2_;
      auto heap = minHeap1_;
      if (!firsts_turn) {
        cost = cost2_;
        other_cost = cost1_;
        heap = minHeap2_;
      }
      auto stop = run(w, r, max, blocked, cost, other_cost, heap);
      if(stop){
        break;
      }
      firsts_turn = !firsts_turn;
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
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;

};  // struct a_star_bi

}  // namespace osr