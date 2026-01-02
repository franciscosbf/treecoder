#include <filesystem>

#include "file.hpp"

namespace file {
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

  Container in(size);

  istream.read(reinterpret_cast<char *>(in.getData()), size);

  return in;
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
  ostream.write(reinterpret_cast<char *>(out.getData()), out.getSize());
}
} // namespace file
