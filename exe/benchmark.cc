#include <filesystem>
#include <thread>
#include <vector>

#include "boost/asio/executor_work_guard.hpp"
#include "boost/asio/io_context.hpp"

#include "fmt/core.h"
#include "fmt/std.h"

#include "conf/options_parser.h"

#include "utl/timer.h"

#include "osr/lookup.h"
#include "osr/routing/profiles/car.h"
#include "osr/routing/route.h"
#include "osr/ways.h"

#include "boost/algorithm/string.hpp"
#include "boost/asio/post.hpp"
#include "boost/beast/version.hpp"
#include "boost/json.hpp"
#include "osr/geojson.h"

namespace json = boost::json;
namespace fs = std::filesystem;
using namespace osr;

class settings : public conf::configuration {
public:
  explicit settings() : configuration("Options") {
    param(data_dir_, "data,d", "Data directory");
    param(threads_, "threads,t", "Number of routing threads");
    param(n_queries_, "n", "Number of queries");
    param(max_dist_, "r", "Radius");
    param(algorithm_, "algorithm",
          "Routing algorithm (dijkstra, astar, bidir)");
    param(location_file_, "locations", "Path to file with locations to parse");
  }

  fs::path data_dir_{"osr"};
  unsigned n_queries_{50};
  unsigned max_dist_{1200};
  unsigned threads_{std::thread::hardware_concurrency()};
  std::string algorithm_{"dijkstra"};
  fs::path location_file_{"/locations.json"};
};

std::vector<location> parse_locations_from_file(const fs::path file_name) {
  std::vector<location> locations;

  // Read the entire JSON file into a string
  std::ifstream input_file(file_name);
  if (!input_file.is_open()) {
    std::cerr << "Could not open the file!" << std::endl;
    return locations;  // Return an empty vector if file can't be opened
  }

  std::string json_str((std::istreambuf_iterator<char>(input_file)),
                       std::istreambuf_iterator<char>());

  // Parse the JSON content using Boost.JSON
  json::value json_data = json::parse(json_str);

  // Loop through the queries and extract locations
  for (const auto& query : json_data.as_object().at("queries").as_array()) {
    // Parse start point
    location start_loc;
    start_loc.pos_.lat_ =
        query.as_object().at("start_point").as_object().at("lat").as_double();
    start_loc.pos_.lng_ =
        query.as_object().at("start_point").as_object().at("lng").as_double();
    locations.push_back(start_loc);

    // Parse end point
    location end_loc;
    end_loc.pos_.lat_ =
        query.as_object().at("end_point").as_object().at("lat").as_double();
    end_loc.pos_.lng_ =
        query.as_object().at("end_point").as_object().at("lng").as_double();
    locations.push_back(end_loc);
  }
  return locations;  // Return the vector of locations
}

int main(int argc, char const* argv[]) {
  auto opt = settings{};
  auto parser = conf::options_parser({&opt});
  parser.read_command_line_args(argc, argv);

  if (parser.help()) {
    parser.print_help(std::cout);
    return 0;
  } else if (parser.version()) {
    return 0;
  }

  parser.read_configuration_file();
  parser.print_unrecognized(std::cout);
  parser.print_used(std::cout);

  if (!fs::is_directory(opt.data_dir_)) {
    fmt::println("directory not found: {}", opt.data_dir_);
    return 1;
  }

  auto const w = ways{opt.data_dir_, cista::mmap::protection::READ};
  lookup const& l = lookup(w);
  bitvec<node_idx_t> const* blocked = nullptr;
  auto const locations = parse_locations_from_file(opt.location_file_);

  auto timer = utl::scoped_timer{"timer"};
  auto threads = std::vector<std::thread>(std::max(1U, opt.threads_));
  auto i = std::atomic_size_t{0U};
  auto j = std::atomic_size_t{0U};
  for (auto& t : threads) {
    t = std::thread([&]() {
      auto h = cista::BASE_HASH;
      auto n1 = 0U;
      if (opt.algorithm_ == "astar") {
        auto a_star_routing = a_star<car>{};
        while (i.fetch_add(1U) < opt.n_queries_) {
          auto local_j =
              j.fetch_add(2U);  // Fetch and increment j by 2 for each thread

          // Read two consecutive entries from locations for 'from' and 'to'
          if (local_j + 1 >= locations.size()) {
            break;  // Prevent out-of-bounds access
          }

          location from = locations[local_j];
          from.lvl_ = level_t::invalid();

          location to = locations[local_j + 1];
          to.lvl_ = level_t::invalid();

          auto const from_match =
              l.match<car>(from, false, direction::kForward, 100, blocked);
          auto const to_match =
              l.match<car>(to, true, direction::kForward, 100, blocked);
          auto p = route(w, l, a_star_routing, from, to, opt.max_dist_,
                         direction::kForward, 100, blocked);
        }
      } else if (opt.algorithm_ == "bidir") {
        auto b = a_star_bi<car>{};
        while (i.fetch_add(1U) < opt.n_queries_) {
          auto local_j =
              j.fetch_add(2U);  // Fetch and increment j by 2 for each thread

          // Read two consecutive entries from locations for 'from' and 'to'
          if (local_j + 1 >= locations.size()) {
            break;  // Prevent out-of-bounds access
          }

          location from = locations[local_j];
          from.lvl_ = level_t::invalid();

          location to = locations[local_j + 1];
          to.lvl_ = level_t::invalid();

          auto const from_match =
              l.match<car>(from, false, direction::kForward, 100, blocked);
          auto const to_match =
              l.match<car>(to, true, direction::kForward, 100, blocked);
          auto p = route(w, l, b, from, to, opt.max_dist_, direction::kForward,
                         100, blocked);
        }
      } else {
        auto d = dijkstra<car>{};
        while (i.fetch_add(1U) < opt.n_queries_) {
          auto local_j =
              j.fetch_add(2U);  // Fetch and increment j by 2 for each thread

          // Read two consecutive entries from locations for 'from' and 'to'
          if (local_j + 1 >= locations.size()) {
            break;  // Prevent out-of-bounds access
          }

          location from = locations[local_j];
          from.lvl_ = level_t::invalid();

          location to = locations[local_j + 1];
          to.lvl_ = level_t::invalid();

          auto const from_match =
              l.match<car>(from, false, direction::kForward, 100, blocked);
          auto const to_match =
              l.match<car>(to, true, direction::kForward, 100, blocked);
          auto p = route(w, l, d, from, to, opt.max_dist_, direction::kForward,
                         100, blocked);
        }
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }
}