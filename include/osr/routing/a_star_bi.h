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

  void add_start() {}

  void reset() {}

  cost_t heuristic() {}

  struct node_h {
    cost_t priority() const {
      return static_cast<std::uint16_t>(cost + heuristic);
    }
    bool operator>(const node_h& other) const {
      return this->priority() > other.priority();
    }
    Profile::label l;
    cost_t cost;
    cost_t heuristic;
  };

  cost_t get_cost() const {}

  template <direction SearchDir, bool WithBlocked>
  void run() {}

  void run() {}

};  // struct bidirectional_a_star

}  // namespace osr