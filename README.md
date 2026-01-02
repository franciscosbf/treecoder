# TreeCoder

A file encoder and decoder which implements the Huffman encoding algorithm.

# Examples

#### Help Message

```text
$ treecoder --help
Usage: treecoder [--help] [--version] file

Positional arguments:
  file           file to decode (*.hff) or encode (without suffix) [required]

Optional arguments:
  -h, --help     shows help message and exits 
  -v, --version  prints version information and exits 
```

#### Encode

```text
$ treecoder file.txt
```


#### Decode

```text
$ treecoder file.hff
```
