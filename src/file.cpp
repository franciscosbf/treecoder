#include <filesystem>

#include "file.hpp"

namespace file {
Container::Container() : data(nullptr), size(0) {}

Container::Container(std::uint8_t *data, std::uint32_t size)
    : data(data), size(size) {}

Container::~Container() {
  if (data == nullptr)
    return;

  delete[] data;
}

const std::uint8_t *Container::getData() const { return data; }

std::uint32_t Container::getSize() const { return size; }

InputContainer::InputContainer() {}

InputContainer::InputContainer(std::uint8_t *data, std::uint32_t size)
    : Container(data, size) {}

InputContainer::~InputContainer() {}

std::uint32_t InputContainer::isEmpty() const { return size == 0; }

InputFile::InputFile(const std::string &filename) : filename(filename) {
  istream.open(filename, std::ifstream::in | std::ifstream::binary);
}

InputFile::~InputFile() {
  if (istream.is_open())
    istream.close();
}

InputContainer InputFile::read() {
  auto size = std::filesystem::file_size(filename);
  if (size == 0)
    return {};

  auto data = new std::uint8_t[size];

  istream.read(reinterpret_cast<char *>(data), size);

  return InputContainer(data, size);
}

OutputContainer::OutputContainer() {}

OutputContainer::OutputContainer(std::uint8_t *data, std::uint32_t size)
    : Container(data, size) {}

OutputContainer::~OutputContainer() {}

OutputFile::OutputFile(const std::string &filename) {
  ostream.open(filename, std::ofstream::out | std::ofstream::binary |
                             std::ofstream::trunc);
}

OutputFile::~OutputFile() {
  if (ostream.is_open())
    ostream.close();
}

void OutputFile::write(const OutputContainer &out) {
  ostream.write(reinterpret_cast<const char *>(out.getData()), out.getSize());
}
} // namespace file
