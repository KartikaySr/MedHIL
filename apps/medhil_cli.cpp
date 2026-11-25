#include "medhil/sim/world.hpp"
#include <charconv>
#include <fstream>
#include <iostream>
#include <map>
using namespace medhil;
namespace {
template <class T> T number(const std::string &s) {
    T value{};
    auto r = std::from_chars(s.data(), s.data() + s.size(), value);
    if (r.ec != std::errc{} || r.ptr != s.data() + s.size())
        throw std::invalid_argument("invalid integer: " + s);
    return value;
}
} // namespace
int main(int argc, char **argv) {
    try {
        const std::map<std::string, Scenario> scenarios = {
            {"nominal", Scenario::Nominal},           {"occlusion_fault", Scenario::Occlusion},
            {"fixture_violation", Scenario::Fixture}, {"monte_carlo", Scenario::MonteCarlo},
            {"force_overload", Scenario::Force},      {"stale_tracker", Scenario::Stale},
            {"joint_limit", Scenario::Joint},         {"hardware_fault", Scenario::Hardware}};
        std::string scenario = "nominal", path;
        std::uint64_t seed = 42;
        unsigned runs = 250;
        for (int i = 1; i < argc; ++i) {
            std::string key = argv[i];
            if (key == "--help") {
                std::cout << "medhil_cli --scenario "
                             "nominal|occlusion_fault|fixture_violation|monte_carlo|force_overload|"
                             "stale_tracker|joint_limit|hardware_fault [--seed N] [--runs N] "
                             "[--output file.csv]\n";
                return 0;
            }
            if (i + 1 == argc)
                throw std::invalid_argument("missing value for " + key);
            std::string v = argv[++i];
            if (key == "--scenario")
                scenario = v;
            else if (key == "--runs")
                runs = number<unsigned>(v);
            else if (key == "--seed")
                seed = number<std::uint64_t>(v);
            else if (key == "--output")
                path = v;
            else
                throw std::invalid_argument("unknown option: " + key);
        }
        if (!scenarios.count(scenario) || !runs)
            throw std::invalid_argument("unknown scenario or empty run count");
        auto selected = scenarios.at(scenario);
        if (selected != Scenario::MonteCarlo)
            runs = 1;
        std::ofstream file;
        if (!path.empty()) {
            file.exceptions(std::ios::failbit | std::ios::badbit);
            file.open(path);
        }
        unsigned passed = 0;
        for (unsigned i = 0; i < runs; ++i) {
            auto r = run(selected, seed + i, file.is_open() ? &file : nullptr, i);
            passed += r.passed(selected);
            if (runs == 1 || !r.passed(selected))
                std::cout << "seed=" << seed + i << " state=" << name(r.state)
                          << " reason=" << name(r.reason) << " violation_us=" << r.violation_us
                          << " brake_us=" << r.brake_us
                          << " final_error_mm=" << r.final_error_m * 1000 << '\n';
        }
        if (file.is_open())
            file.close();
        std::cout << "scenario=" << scenario << " passed=" << passed << '/' << runs << '\n';
        return passed == runs ? 0 : 1;
    } catch (const std::exception &e) {
        std::cerr << "medhil_cli: " << e.what() << '\n';
        return 2;
    }
}
