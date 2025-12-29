#pragma once

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

class Output {
private:
  std::uint8_t *data;
  std::uint32_t size;

public:
  Output();

  Output(std::uint8_t *data, std::uint32_t size);

  ~Output();

  const std::uint8_t *getData() const;

  std::uint32_t getSize() const;
};

class OutputFile {
private:
  std::ofstream ostream;

public:
  OutputFile(const std::string &filename);

  ~OutputFile();

  void write(const Output &out);
};
} // namespace file
