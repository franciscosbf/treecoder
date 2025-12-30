#pragma once

#include <cstdint>
#include <fstream>
#include <string>

namespace file {
class Container {
protected:
  std::uint8_t *data;
  std::uint32_t size;

  Container();

  Container(std::uint8_t *data, std::uint32_t size);

public:
  virtual ~Container();

  const std::uint8_t *getData() const;

  std::uint32_t getSize() const;
};

class InputContainer final : public Container {
public:
  InputContainer();

  InputContainer(std::uint8_t *data, std::uint32_t size);

  ~InputContainer();

  std::uint32_t isEmpty() const;
};

class InputFile {
private:
  const std::string &filename;
  std::ifstream istream;

public:
  InputFile(const std::string &filename);

  ~InputFile();

  InputContainer read();
};

class OutputContainer final : public Container {
public:
  OutputContainer();

  OutputContainer(std::uint8_t *data, std::uint32_t size);

  ~OutputContainer();
};

class OutputFile {
private:
  std::ofstream ostream;

public:
  OutputFile(const std::string &filename);

  ~OutputFile();

  void write(const OutputContainer &out);
};
} // namespace file
