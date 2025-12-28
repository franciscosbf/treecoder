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

OutputFile::OutputFile(const std::string &filename) {
  ostream.open(filename, std::ofstream::out | std::ofstream::binary |
                             std::ofstream::trunc);
}

OutputFile::~OutputFile() {
  if (ostream.is_open())
    ostream.close();
}

void OutputFile::write(const std::vector<std::uint8_t> &content) {
  ostream.write(reinterpret_cast<const char *>(content.data()), content.size());
}
} // namespace file
