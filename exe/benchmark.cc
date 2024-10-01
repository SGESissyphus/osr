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
    param(location_file_, "locations", "Path to file with locations to parse");
  }

  fs::path data_dir_{"osr-hessen"};
  unsigned n_queries_{50};
  unsigned max_dist_{1200};
  unsigned threads_{std::thread::hardware_concurrency()};
  fs::path location_file_{"locations.json"};
};

std::vector<location> parse_locations_from_file(const fs::path file_name) {
  std::vector<location> locations;

  std::ifstream input_file(file_name);
  if (!input_file.is_open()) {
    std::cerr << "Could not open the file!" << std::endl;
    return locations;
  }

  std::string json_str((std::istreambuf_iterator<char>(input_file)),
                       std::istreambuf_iterator<char>());

  json::value json_data = json::parse(json_str);

  for (const auto& query : json_data.as_object().at("queries").as_array()) {
    location start_loc;
    start_loc.pos_.lat_ =
        query.as_object().at("start_point").as_object().at("lat").as_double();
    start_loc.pos_.lng_ =
        query.as_object().at("start_point").as_object().at("lng").as_double();
    locations.push_back(start_loc);

    location end_loc;
    end_loc.pos_.lat_ =
        query.as_object().at("end_point").as_object().at("lat").as_double();
    end_loc.pos_.lng_ =
        query.as_object().at("end_point").as_object().at("lng").as_double();
    locations.push_back(end_loc);
  }
  return locations;
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

  std::atomic_size_t i{0U}, j{0U};

  std::vector<std::thread> threads(std::max(1U, opt.threads_));

  std::mutex print_mutex;  // To synchronize output

  for (auto& t : threads) {
    t = std::thread([&]() {
      while (i.fetch_add(1U) < opt.n_queries_) {
        auto local_j = j.fetch_add(2U);

        if (local_j + 1 >= locations.size()) {
          break;
        }

        location from = locations[local_j];
        from.lvl_ = level_t::invalid();

        location to = locations[local_j + 1];
        to.lvl_ = level_t::invalid();

        // Dijkstra timing
        double dijkstra_time = 0.0, astar_time = 0.0, bidir_time = 0.0;
        {
          auto start_time = std::chrono::steady_clock::now();
          auto dijkstra_routing = dijkstra<car>{};
          auto const from_match =
              l.match<car>(from, false, direction::kForward, 100, blocked);
          auto const to_match =
              l.match<car>(to, true, direction::kForward, 100, blocked);
          auto p = route(w, l, dijkstra_routing, from, to, opt.max_dist_,
                         direction::kForward, 100, blocked);
          auto end_time = std::chrono::steady_clock::now();
          dijkstra_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        }

        // A* timing
        {
          auto start_time = std::chrono::steady_clock::now();
          auto astar_routing = a_star<car>{};
          auto const from_match =
              l.match<car>(from, false, direction::kForward, 100, blocked);
          auto const to_match =
              l.match<car>(to, true, direction::kForward, 100, blocked);
          auto p = route(w, l, astar_routing, from, to, opt.max_dist_,
                         direction::kForward, 100, blocked);
          auto end_time = std::chrono::steady_clock::now();
          astar_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        }

        // Bidirectional A* timing
        {
          auto start_time = std::chrono::steady_clock::now();
          auto bidir_routing = a_star_bi<car>{};
          auto const from_match =
              l.match<car>(from, false, direction::kForward, 100, blocked);
          auto const to_match =
              l.match<car>(to, true, direction::kForward, 100, blocked);
          auto p = route(w, l, bidir_routing, from, to, opt.max_dist_,
                         direction::kForward, 100, blocked);
          auto end_time = std::chrono::steady_clock::now();
          bidir_time = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        }
        {
          std::lock_guard<std::mutex> lock(print_mutex);
          fmt::println("Query {}: Dijkstra Time = {} ms, A* Time = {} ms, Bidirectional A* Time = {} ms",
                       local_j / 2, dijkstra_time, astar_time, bidir_time);
        }
      }
    });
  }

  for (auto& t : threads) {
    t.join();
  }

  return 0;
}