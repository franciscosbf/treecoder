#include <filesystem>

#include "file.hpp"

namespace file {
Container::Container() : data(nullptr), size(0) {}

void doNotDelete(std::uint8_t *data) {}

Container::Container(std::uint8_t *data, std::uint32_t size, bool free)
    : size(size) {
  this->data = free ? std::shared_ptr<std::uint8_t>(data)
                    : std::shared_ptr<std::uint8_t>(data, doNotDelete);
}

Container::~Container() {}

const std::uint8_t *Container::getData() const { return data.get(); }

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
