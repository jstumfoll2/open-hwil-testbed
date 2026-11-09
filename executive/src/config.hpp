// Run configuration for the compiled executive.
//
// Python (hwil/cppexec.py) resolves the scenario and writes it as a flat,
// line-based text file so this program needs no JSON library:
//
//   key value                      # scalar settings
//   phase <type> key value ...     # one line per mission phase, in order
//   fault <type> key value ...     # one line per fault
//
// Copyright 2026 The open-hwil-testbed authors. Licensed under Apache-2.0.
#pragma once

#include <map>
#include <string>
#include <vector>

namespace hwil {

struct Item {
  std::string type;
  std::map<std::string, double> num;

  double get(const std::string& k, double fallback) const {
    auto it = num.find(k);
    return it == num.end() ? fallback : it->second;
  }
  bool has(const std::string& k) const { return num.count(k) != 0; }
};

struct Config {
  std::map<std::string, std::string> kv;
  std::vector<Item> mission;
  std::vector<Item> faults;

  std::string str(const std::string& k) const;
  double num(const std::string& k) const;
  double num(const std::string& k, double fallback) const;
};

Config load_config(const std::string& path);

}  // namespace hwil
