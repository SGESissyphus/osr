#pragma once

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
    if (cost1_[start.get_node().get_key()].update(
            start, start.get_node(), start.cost(), node::invalid())) {
      minHeap1_.push_back(node_h{start, 0, 0});
    }
    if (cost2_[end.get_node().get_key()].update(end, end.get_node(), end.cost(),
                                                node::invalid())) {
      minHeap2_.push_back(node_h{end, 0, 0});
    }
  }

  void reset(cost_t,
             location const& end_loc,
             location const& start_loc,
             std::vector<way_candidate> to_match) {
    minHeap1_.clear();
    minHeap2_.clear();
    cost1_.clear();
    cost2_.clear();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    to_match_ = to_match;
  }

  cost_t heuristic() {}

  cost_t get_cost() const {}

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked,
           unordered_dense::map<key, entry, hash> cost,
           std::vector<node_h> heap,
           std::vector<node_h> other_heap) {
    std::make_heap(heap.begin(), heap.end(), std::greater<node_h>{});
    std::pop_heap(heap.begin(), heap.end(), std::greater<node_h>{});
    auto curr_node_h = heap.back();

    // abbruchbedingung

    auto l = curr_node_h.l;
    if (get_cost(l.get_node()) < l.cost()) {
      continue;
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
            node_h next_h = node_h{next, next.cost_, heuristic(turn, next, w)};
            heap.push_back(next_h);
            std::push_heap(heap.begin(), heap.end(), std::greater<node_h>{});
          }
        });
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked) {
    bool firstsTurn = true;
    while (!minHeap1_.empty() && !minHeap2_.empty()) {

      auto cost;
      auto heap;
      auto other_heap;
      if (firstsTurn) {
        cost = cost1_;
        heap = minHeap1_;
        other_heap = minHeap2_;
      } else {
        cost = cost2_;
        heap = minHeap2_;
        other_heap = minHeap1_;
      }
      auto stop = run(w, r, max, blocked, cost, heap, other_heap);
      firstsTurn = !firstsTurn;
    }
  }

  void run() {}

  location start_loc_;
  location end_loc_;
  match_t to_match_;
  std::vector<node_h> minHeap1_;
  std::vector<node_h> minHeap2_;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;

};  // struct bidirectional_a_star

}  // namespace osr