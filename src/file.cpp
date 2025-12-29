#include <filesystem>
#include <vector>

#include "file.hpp"

namespace file {

InputFile::InputFile(const std::string &filename) : filename(filename) {
  istream.open(filename, std::ifstream::in | std::ifstream::binary);
}

InputFile::~InputFile() {
  if (istream.is_open())
    istream.close();
}

std::vector<std::uint8_t> InputFile::read() {
  auto fsize = std::filesystem::file_size(filename);
  if (fsize == 0)
    return {};
  std::vector<std::uint8_t> content(fsize);

  istream.read(reinterpret_cast<char *>(content.data()), fsize);

  return content;
}

Output::Output() : data(nullptr), size(0) {}

Output::Output(std::uint8_t *data, std::uint32_t size)
    : data(data), size(size) {}

Output::~Output() { delete[] data; }

const std::uint8_t *Output::getData() const { return data; }

std::uint32_t Output::getSize() const { return size; }

OutputFile::OutputFile(const std::string &filename) {
  ostream.open(filename, std::ofstream::out | std::ofstream::binary |
                             std::ofstream::trunc);
}

OutputFile::~OutputFile() {
  if (ostream.is_open())
    ostream.close();
}

void OutputFile::write(const Output &out) {
  ostream.write(reinterpret_cast<const char *>(out.getData()), out.getSize());
}
} // namespace file
