// Copyright 2026 The open-hwil-testbed authors. Licensed under Apache-2.0.
#include "config.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace hwil {

std::string Config::str(const std::string& k) const {
  auto it = kv.find(k);
  if (it == kv.end()) throw std::runtime_error("config: missing key '" + k + "'");
  return it->second;
}

double Config::num(const std::string& k) const { return std::stod(str(k)); }

double Config::num(const std::string& k, double fallback) const {
  auto it = kv.find(k);
  return it == kv.end() ? fallback : std::stod(it->second);
}

static Item parse_item(std::istringstream& in) {
  Item it;
  in >> it.type;
  std::string key;
  double val;
  while (in >> key >> val) it.num[key] = val;
  return it;
}

Config load_config(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot open config " + path);
  Config c;
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::istringstream in(line);
    std::string key;
    in >> key;
    if (key == "phase") {
      c.mission.push_back(parse_item(in));
    } else if (key == "fault") {
      c.faults.push_back(parse_item(in));
    } else {
      std::string rest;
      std::getline(in >> std::ws, rest);
      c.kv[key] = rest;
    }
  }
  return c;
}

}  // namespace hwil
