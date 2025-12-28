#pragma once

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

namespace file {

class InputFile {
private:
  const std::string &filename;
  std::ifstream istream;

public:
  InputFile(const std::string &filename);

  ~InputFile();

  std::vector<std::uint8_t> read();
};

class OutputFile {
private:
  std::ofstream ostream;

public:
  OutputFile(const std::string &filename);

  ~OutputFile();

  void write(const std::vector<std::uint8_t> &content);
};
} // namespace file
