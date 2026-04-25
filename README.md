# TreeCoder

A file encoder and decoder which implements the Huffman encoding algorithm.

# Examples

#### Help Message

```text
treecoder --help
Usage: treecoder [--help] [--version] file

Positional arguments:
  file           file to decode (*.hff) or encode (without suffix) [required]

Optional arguments:
  -h, --help     shows help message and exits 
  -v, --version  prints version information and exits 
```

#### Encode

```text
treecoder file.txt
```

#### Decode

```text
treecoder file.hff
```

# Compilation Notes

As far as I known, there aren't any shenanigans. I have empiric guarantees that this compiles on x86-64 and arm64 (apple silicon chip), so we good!

#### External Dependencies

Besides most dependencies being fetched directly, treecoder requires [OpenSSL](https://www.openssl.org/) installed and the [endian](https://github.com/steinwurf/endian) submodule. The latter can be configured simply by executing:

```sh
git submodule init && git submodule update
```

Regarding OpenSSL, e.g., Ubuntu doesn't come with the development package. Therefore, you would need to:

```sh
sudo apt update && sudo apt install openssl-dev
```

This is just to say that not all operating systems include the development library by default.

#### Compiling Everything

```text
# e.g., Build type set to Release
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
```

The resulting `build` folder will contain `treecoder` (program) and `treecoder_test` (tests).
