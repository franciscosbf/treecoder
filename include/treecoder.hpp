#pragma once

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#define BITS_IN_BYTE 8

namespace treecoder {

class TreeCoderError : public std::exception {
private:
  std::string reason;

public:
  TreeCoderError(std::string reason);

  const char *what() const noexcept override;
};

std::unordered_map<std::uint8_t, std::size_t>
computeFrequencyTable(const std::vector<std::uint8_t> &raw);

class HuffmanNode {
private:
  std::uint32_t weight;

protected:
  HuffmanNode(std::uint32_t weight);

public:
  virtual ~HuffmanNode() = default;

  virtual bool isLeaf() const = 0;

  std::uint32_t getWeight() const;

  template <typename N = HuffmanNode> const N &downcast() const {
    static_assert(std::is_base_of<HuffmanNode, N>::value,
                  "N is not subclass of HuffmanNode");

    return static_cast<const N &>(*this);
  }
};

class HuffmanInternalNode final : public HuffmanNode {
private:
  std::shared_ptr<HuffmanNode> left_node, right_node;

public:
  HuffmanInternalNode(std::shared_ptr<HuffmanNode> left_node,
                      std::shared_ptr<HuffmanNode> right_node, int weight);

  ~HuffmanInternalNode();

  bool isLeaf() const;

  const HuffmanNode &getLeftNode() const;

  const HuffmanNode &getRightNode() const;
};

class HuffmanLeafNode final : public HuffmanNode {
private:
  std::uint8_t byte;

public:
  HuffmanLeafNode(std::uint8_t b, std::uint32_t weight);

  ~HuffmanLeafNode();

  bool isLeaf() const;

  std::uint8_t getByte() const;
};

class HuffmanTree {
private:
  std::shared_ptr<HuffmanNode> root_node;

public:
  HuffmanTree(std::uint8_t byte, std::uint32_t weight);

  HuffmanTree(std::shared_ptr<HuffmanTree> left_tree,
              std::shared_ptr<HuffmanTree> right_tree);

  ~HuffmanTree();

  const HuffmanNode &getRoot() const;

  static std::shared_ptr<HuffmanTree>
  build(const std::unordered_map<std::uint8_t, std::size_t> &frequencies);
};

class PrefixCodeEntry {
private:
  std::size_t frequency;
  std::uint8_t code, bits;

public:
  PrefixCodeEntry(std::size_t frequency, std::uint8_t code, std::uint8_t bits);

  std::size_t getFrequency() const;

  std::uint8_t getCode() const;

  std::uint8_t getBits() const;
};

void computePrefixCodePerByte(
    const HuffmanNode &node,
    std::unordered_map<std::uint8_t, PrefixCodeEntry> &prefix_table,
    std::uint32_t code = 0, std::uint8_t bits = 0);

std::unordered_map<std::uint8_t, PrefixCodeEntry>
computePrefixCodeTable(const std::shared_ptr<HuffmanTree> tree);

std::vector<std::uint8_t> encodePrefixTableAndInput(
    const std::unordered_map<std::uint8_t, PrefixCodeEntry> &table,
    const std::vector<std::uint8_t> &in);

std::vector<std::uint8_t>
decodePrefixTableAndInput(const std::vector<std::uint8_t> &in);

class TreeCoder {
public:
  TreeCoder();

  ~TreeCoder();

  std::vector<std::uint8_t> encode(const std::vector<std::uint8_t> &in);

  std::vector<std::uint8_t> decode(const std::vector<std::uint8_t> &in);
};
} // namespace treecoder
