#pragma once

#include <cstdint>
#include <fstream>
#include <string>

namespace file {
class Container {
protected:
  std::uint8_t *data;
  std::uint32_t size;
  bool free;

public:
  Container();

  Container(std::uint8_t *data, std::uint32_t size, bool free = true);

  ~Container();

  Container(const Container &c) {
    const_cast<Container &>(c).free = false;
    data = c.data;
    size = c.size;
  }

  const std::uint8_t *getData() const;

  std::uint32_t getSize() const;

  std::uint32_t isEmpty() const;
};

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
