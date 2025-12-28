#include <argparse/argparse.hpp>
#include <cstddef>
#include <cstdlib>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "file.hpp"
#include "treecoder.hpp"
#include "version.hpp"

using namespace argparse;
using namespace treecoder;
using namespace file;

const std::string HUFFMAN_SUFFIX = ".hff";

bool wants_to_decode(const std::string &ifilename) {
  return ifilename.size() >= HUFFMAN_SUFFIX.size() &&
         ifilename.compare(ifilename.size() - HUFFMAN_SUFFIX.size(),
                           HUFFMAN_SUFFIX.size(), HUFFMAN_SUFFIX) == 0;
}

int main(int argc, char **argv) {
  ArgumentParser program(PROGRAM_NAME, PROGRAM_VERSION);

  program.add_argument("file").required().help(
      "file to decode (*" + HUFFMAN_SUFFIX + ") or encode (without suffix)");

  try {
    program.parse_args(argc, argv);
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl << program;

    return EXIT_FAILURE;
  }

  const std::string ifilename = program.get("file");
  std::vector<std::uint8_t> icontent;

  try {
    InputFile ifile(ifilename);
    icontent = ifile.read();
  } catch (const std::exception &e) {
    std::cerr << "While reading input file " << ifilename << ": " << e.what()
              << std::endl;

    return EXIT_FAILURE;
  }

  bool to_decode = wants_to_decode(ifilename);
  std::string ofilename;
  std::vector<std::uint8_t> ocontent;
  TreeCoder tc;
  std::function<void()> treecode;

  if (to_decode) {
    ofilename = ifilename.substr(ifilename.size() - HUFFMAN_SUFFIX.size());

    treecode = [&]() { ocontent = tc.decode(icontent); };
  } else {
    ofilename = ifilename + HUFFMAN_SUFFIX;

    treecode = [&]() { ocontent = tc.encode(icontent); };
  }

  try {
    treecode();
  } catch (const std::exception &e) {
    std::cerr << "While " << (to_decode ? "decoding " : "encoding") << ifilename
              << ": " << e.what() << std::endl;

    return EXIT_FAILURE;
  }

  try {
    OutputFile ofile(ofilename);
    ofile.write(ocontent);
  } catch (const std::exception &e) {
    std::cerr << "While writing output file " << ifilename << ": " << e.what();
  }

  return EXIT_SUCCESS;
}
