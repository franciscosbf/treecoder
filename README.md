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

#### Compiling Everything

```text
cmake -B build && cmake --build build
```

The resulting `build` folder will contain `treecoder` (program) and `treecoder_test` (tests).
