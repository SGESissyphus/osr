#pragma once

#include "geo/webmercator.h"
#include "osr/routing/dial.h"
#include "osr/types.h"
#include "osr/ways.h"

namespace osr {

template <typename Profile>
struct a_star {
  using profile_t = Profile;
  using key = typename Profile::key;
  using label = typename Profile::label;
  using node = typename Profile::node;
  using entry = typename Profile::entry;
  using hash = typename Profile::hash;

  void add_start(label const l, ways const& w) {
    if (cost_[l.get_node().get_key()].update(l, l.get_node(), l.cost(),
                                             node::invalid())) {
      pq_.push(node_h{l, 0, heuristic(l, w)});
    }
  }

  void reset(cost_t max,
             location const& end_loc,
             std::vector<way_candidate> const& to_match) {
    pq_.clear();
    pq_.n_buckets(max + 1U);

    cost_.clear();
    end_loc_ = end_loc;
    to_match_ = to_match;
  }

  cost_t heuristic(label const l, ways const& w) {
    auto const start_coord =
        geo::latlng_to_merc(w.get_node_pos(l.n_).as_latlng());
    auto const end_node = geo::latlng_to_merc(end_loc_.pos_);

    auto const dx = start_coord.x_ - end_node.x_;
    auto const dy = start_coord.y_ - end_node.y_;

    auto const dist = std::sqrt(dx * dx + dy * dy);

    return Profile::heuristic(dist);
  };

  /*double newtonSqrt(double x) {
      double x1 = x;
      double x2 = x / 2;
      while (std::abs(x1 - x2) >= 0.0001) {
        x1 = x2;
        x2 = (x1 + x / x1) / 2;
      }
      return x2;
    }*/

  struct node_h {
    cost_t priority() const {
      return static_cast<std::uint16_t>(cost + heuristic);
    }
    bool operator<(const node_h& other) const {
      return this->priority() < other.priority();
    }
    bool operator>(const node_h& other) const {
      return this->priority() > other.priority();
    }
    Profile::label l;
    cost_t cost;
    cost_t heuristic;
  };

  struct get_bucket {
    cost_t operator()(node_h const& n) { return n.cost + n.heuristic; }
  };

  cost_t get_cost(node const n) const {
    auto const it = cost_.find(n.get_key());
    return it != end(cost_) ? it->second.cost(n) : kInfeasible;
  }

  template <direction SearchDir, bool WithBlocked>
  void run(ways const& w,
           ways::routing const& r,
           cost_t const max,
           bitvec<node_idx_t> const* blocked) {

    auto buffer = Profile::get_static_buffer();

    bool found = false;

    while (!pq_.empty() && !to_match_.empty() && buffer > 0) {
      auto curr_node_h = pq_.pop();
      auto l = curr_node_h.l;

      if (!found) {
        buffer = buffer + Profile::get_dynamic_buffer();
        for (auto const& dest : to_match_) {
          if (l.n_ == dest.right_.node_) {
            found = true;
          } else if (l.n_ == dest.left_.node_) {
            found = true;
          }
        }
      } else {
        buffer--;
      }

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
                cost_[neighbor.get_key()].update(
                    l, neighbor, static_cast<cost_t>(total), curr)) {
              auto next = label{neighbor, static_cast<cost_t>(total)};
              next.track(l, r, way, neighbor.get_node());
              node_h next_h = node_h{next, next.cost_, heuristic(next, w)};
              if (next_h.cost + next_h.heuristic < max) {
                pq_.push(next_h);
              }
            }
          });
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
  dial<node_h, get_bucket> pq_{get_bucket{}};
  std::optional<label> end_node_label;
  location end_loc_;
  match_t to_match_;
  ankerl::unordered_dense::map<key, entry, hash> cost_;
};
}  // namespace osr
