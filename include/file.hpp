#pragma once

#include <fstream>
#include <string>

#include "container.hpp"

using namespace container;

namespace file {
class InputFile {
private:
  const std::string &filename;
  std::ifstream istream;

public:
  InputFile(const std::string &filename);

  ~InputFile();

  Container read();
};

class OutputFile {
private:
  std::ofstream ostream;

public:
  OutputFile(const std::string &filename);

  ~OutputFile();

  void write(const Container &out);
};
} // namespace file
