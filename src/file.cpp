#include <filesystem>

#include "file.hpp"

namespace file {
Container::Container() : data(nullptr), size(0) {}

Container::Container(std::uint8_t *data, std::uint32_t size, bool free)
    : data(data), size(size), free(free) {}

Container::~Container() {
  if (data == nullptr || !free)
    return;

  delete[] data;
}

const std::uint8_t *Container::getData() const { return data; }

std::uint32_t Container::getSize() const { return size; }

std::uint32_t Container::isEmpty() const { return size == 0; }

InputFile::InputFile(const std::string &filename) : filename(filename) {
  istream.open(filename, std::ifstream::in | std::ifstream::binary);
}

InputFile::~InputFile() {
  if (istream.is_open())
    istream.close();
}

Container InputFile::read() {
  auto size = std::filesystem::file_size(filename);
  if (size == 0)
    return {};

  auto data = new std::uint8_t[size];

  istream.read(reinterpret_cast<char *>(data), size);

  return Container(data, size);
}

OutputFile::OutputFile(const std::string &filename) {
  ostream.open(filename, std::ofstream::out | std::ofstream::binary |
                             std::ofstream::trunc);
}

OutputFile::~OutputFile() {
  if (ostream.is_open())
    ostream.close();
}

void OutputFile::write(const Container &out) {
  ostream.write(reinterpret_cast<const char *>(out.getData()), out.getSize());
}
} // namespace file
