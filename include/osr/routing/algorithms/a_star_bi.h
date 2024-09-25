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
    std::cout << "add in heap1: " << static_cast<std::uint32_t>(l.n_)
              << std::endl;
    if (cost1_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                              node::invalid())) {
      minHeap1_.push_back(node_h{l, 0, heuristic_to_end(l, w)});
    }
  }

  void add_end(label const l, ways const& w) {
    std::cout << "add in heap2: " << static_cast<std::uint32_t>(l.n_)
              << std::endl;
    if (cost2_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                              node::invalid())) {
      minHeap2_.push_back(node_h{l, 0, heuristic_to_start(l, w)});
    }
  }

  void clear_meetpoint() { meet_point = meet_point.invalid(); }

  void reset(cost_t,
             location const& start_loc,
             location const& end_loc,
             match_t start,
             match_t end) {
    minHeap1_.clear();
    minHeap2_.clear();
    cost1_.clear();
    cost2_.clear();
    expanded_.clear();
    meet_point.invalid();
    start_loc_ = start_loc;
    end_loc_ = end_loc;
    start_ = std::move(start);
    end_ = std::move(end);
  }

  cost_t heuristic_to_end(label const l, ways const& w) {
    auto const node_coord =
        geo::latlng_to_merc(w.get_node_pos(l.n_).as_latlng());
    auto const end_node = geo::latlng_to_merc(end_loc_.pos_);

    auto const dx = node_coord.x_ - end_node.x_;
    auto const dy = node_coord.y_ - end_node.y_;

    auto const dist = newtonSqrt(dx * dx + dy * dy);

    return dist / to_meters_per_second(speed_limit::kmh_120);
  }

  cost_t heuristic_to_start(label const l, ways const& w) {
    auto const node_coord =
        geo::latlng_to_merc(w.get_node_pos(l.n_).as_latlng());
    auto const start_node = geo::latlng_to_merc(start_loc_.pos_);

    auto const dx = node_coord.x_ - start_node.x_;
    auto const dy = node_coord.y_ - start_node.y_;

    auto const dist = newtonSqrt(dx * dx + dy * dy);

    return dist / to_meters_per_second(static_cast<speed_limit>(5U));
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
  cost_t get_cost_from_start(node const n) const {
    // std::cout << "before find in get_cost_from\n";
    auto const it = cost1_.find(n.get_key());
    // std::cout << "after find in get_cost_from\n";
    return it != end(cost1_) ? it->second.cost(n) : kInfeasible;
  }

  cost_t get_cost_from_end(node const n) const {
    // std::cout << "before find in get_cost_end\n";
    auto const it = cost2_.find(n.get_key());
    // std::cout << "after find in get_cost_end\n";
    return it != end(cost2_) ? it->second.cost(n) : kInfeasible;
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked) {
    std::cout << "run begin\n";

    std::make_heap(minHeap1_.begin(), minHeap1_.end(), std::greater<node_h>{});
    std::make_heap(minHeap2_.begin(), minHeap2_.end(), std::greater<node_h>{});

    while (!minHeap1_.empty() && !minHeap2_.empty()) {
      std::cout << "in the while-loop run \n";
      auto curr1 = run_start_to_end<SearchDir, WithBlocked>(w, r, max, blocked);
      auto curr2 = run_end_to_start<SearchDir, WithBlocked>(w, r, max, blocked);
      std::cout << "after finding currs\n";
      if (curr1 != std::nullopt) {
        if (!expanded_.contains(curr1.value())) {
          std::cout << "curr1 adds in expand: "
                    << static_cast<std::uint32_t>(curr1.value().n_)
                    << std::endl;
          expanded_.emplace(curr1.value());
        } else {
          meet_point = curr1.value();
          std::cout << "breaking because curr1 is already expanded:"
                    << static_cast<std::uint32_t>(meet_point.n_) << std::endl;
          return;
        }
      }
      if (curr2 != std::nullopt) {
        if (!expanded_.contains(curr2.value())) {
          std::cout << "curr2 adds in expand: "
                    << static_cast<std::uint32_t>(curr2.value().n_)
                    << std::endl;
          expanded_.emplace(curr2.value());
        } else {
          meet_point = curr2.value();
          std::cout << "breaking because curr2 is already expanded:"
                    << static_cast<std::uint32_t>(meet_point.n_) << std::endl;
          return;
        }
      }
    }
  }

  template <direction SearchDir, bool WithBlocked>
  std::optional<node> run_start_to_end(ways const& w,
                                       ways::routing const& r,
                                       cost_t const max,
                                       bitvec<node_idx_t> const* blocked) {
    // std::cout << "run start to end \n";
    std::pop_heap(minHeap1_.begin(), minHeap1_.end(), std::greater<node_h>{});
    auto curr_node_h = minHeap1_.back();
    minHeap1_.pop_back();

    auto l = curr_node_h.l;
    auto curr = l.get_node();

    if (get_cost_from_start(curr) < l.cost()) {
      return std::nullopt;  // TODO check for good return value
    }

    Profile::template adjacent<SearchDir, WithBlocked>(
        r, curr, blocked,
        [&](node const neighbor, std::uint32_t const cost, distance_t,
            way_idx_t const way, std::uint16_t, std::uint16_t) {
          auto const total = l.cost() + cost;
          if (total < max &&
              cost1_[neighbor.get_key()].update(
                  l, neighbor, static_cast<cost_t>(total), curr)) {
            auto next = label{neighbor, static_cast<cost_t>(total)};
            next.track(l, r, way, neighbor.get_node());
            node_h next_h = node_h{next, next.cost_, heuristic_to_end(next, w)};
            minHeap1_.push_back(next_h);
            std::push_heap(minHeap1_.begin(), minHeap1_.end(),
                           std::greater<node_h>{});
          }
        });
    return curr;
  }

  template <direction SearchDir, bool WithBlocked>
  std::optional<node> run_end_to_start(ways const& w,
                                       ways::routing const& r,
                                       cost_t const max,
                                       bitvec<node_idx_t> const* blocked) {
    // std::cout << "run end to start \n";
    std::pop_heap(minHeap2_.begin(), minHeap2_.end(), std::greater<node_h>{});
    auto curr_node_h = minHeap2_.back();
    minHeap2_.pop_back();

    auto l = curr_node_h.l;
    auto curr = l.get_node();

    if (get_cost_from_end(curr) < l.cost()) {
      return std::nullopt;  // TODO check for good return value
    }
    // std::cout << "before adjecency end-start\n";
    Profile::template adjacent<opposite(SearchDir), WithBlocked>(
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
    // std::cout << "after adjecency end-start\n";
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
  match_t start_;
  match_t end_;
  hash_set<node> expanded_;
  node meet_point;
  ankerl::unordered_dense::map<key, entry, hash> cost1_;
  ankerl::unordered_dense::map<key, entry, hash> cost2_;

};  // struct a_star_bi

}  // namespace osr