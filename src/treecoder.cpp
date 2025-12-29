#include <cassert>
#include <cmath>
#include <cstring>
#include <memory>
#include <queue>

#include "endian/big_endian.hpp"
#include "openssl/sha.h"
#include "prefixtable_generated.h"
#include "treecoder.hpp"

using namespace endian;
using namespace flatbuffers;
using namespace prefixtable;

namespace treecoder {
TreeCoderError::TreeCoderError(std::string reason) : reason(reason) {}

const char *TreeCoderError::what() const noexcept { return reason.c_str(); }

HuffmanNode::HuffmanNode(std::uint32_t weight) : weight(weight) {}

std::uint32_t HuffmanNode::getWeight() const { return weight; }

HuffmanInternalNode::HuffmanInternalNode(
    std::shared_ptr<HuffmanNode> left_node,
    std::shared_ptr<HuffmanNode> right_node, int weight)
    : HuffmanNode(weight), left_node(left_node), right_node(right_node) {}

HuffmanInternalNode::~HuffmanInternalNode() {}

bool HuffmanInternalNode::isLeaf() const { return false; }

const HuffmanNode &HuffmanInternalNode::getLeftNode() const {
  return *left_node.get();
}

const HuffmanNode &HuffmanInternalNode::getRightNode() const {
  return *right_node.get();
}

HuffmanLeafNode::HuffmanLeafNode(std::uint8_t byte, std::uint32_t weight)
    : HuffmanNode(weight), byte(byte) {}

HuffmanLeafNode::~HuffmanLeafNode() {}

bool HuffmanLeafNode::isLeaf() const { return true; }

std::uint8_t HuffmanLeafNode::getByte() const { return byte; }

HuffmanTree::HuffmanTree(std::uint8_t byte, std::uint32_t weight)
    : root_node(std::make_shared<HuffmanLeafNode>(byte, weight)) {}

HuffmanTree::HuffmanTree(std::shared_ptr<HuffmanTree> left_tree,
                         std::shared_ptr<HuffmanTree> right_tree)
    : root_node(std::make_shared<HuffmanInternalNode>(
          left_tree->root_node, right_tree->root_node,
          left_tree->root_node->getWeight() +
              right_tree->root_node->getWeight())) {}

const HuffmanNode &HuffmanTree::getRoot() const { return *root_node.get(); }

struct ComparableHuffmanTree {
  bool operator()(std::shared_ptr<HuffmanTree> left_tree,
                  std::shared_ptr<HuffmanTree> right_tree) {
    return left_tree->getRoot().getWeight() > right_tree->getRoot().getWeight();
  }
};

std::shared_ptr<HuffmanTree> HuffmanTree::build(
    const std::unordered_map<std::uint8_t, std::size_t> &frequencies) {
  assert(!frequencies.empty() &&
         "frequencies table must contain at least one entry");

  std::priority_queue<std::shared_ptr<HuffmanTree>,
                      std::vector<std::shared_ptr<HuffmanTree>>,
                      ComparableHuffmanTree>
      prioritized_frequencies;

  for (const auto &frequency : frequencies)
    prioritized_frequencies.push(
        std::make_shared<HuffmanTree>(frequency.first, frequency.second));

  while (prioritized_frequencies.size() > 1) {
    auto left_tree = prioritized_frequencies.top();
    prioritized_frequencies.pop();

    auto right_tree = prioritized_frequencies.top();
    prioritized_frequencies.pop();

    prioritized_frequencies.push(
        std::make_shared<HuffmanTree>(left_tree, right_tree));
  }

  return prioritized_frequencies.top();
}

HuffmanTree::~HuffmanTree() {}

std::unordered_map<std::uint8_t, std::size_t>
computeFrequencyTable(const std::vector<std::uint8_t> &bytes) {
  std::unordered_map<std::uint8_t, std::size_t> frequencies;

  for (auto b : bytes) {
    if (auto frequency = frequencies.find(b); frequency != frequencies.end())
      frequency->second++;
    else
      frequencies.insert({b, 1});
  }

  return frequencies;
}

PrefixCodeEntry::PrefixCodeEntry(std::size_t frequency, std::uint8_t code,
                                 std::uint8_t bits)
    : frequency(frequency), code(code), bits(bits) {}

std::size_t PrefixCodeEntry::getFrequency() const { return frequency; }

std::uint8_t PrefixCodeEntry::getCode() const { return code; }

std::uint8_t PrefixCodeEntry::getBits() const { return bits; }

void computePrefixCodePerByte(
    const HuffmanNode &node,
    std::unordered_map<std::uint8_t, PrefixCodeEntry> &table,
    std::uint32_t code, std::uint8_t bits) {
  if (node.isLeaf()) {
    auto leaf_node = node.downcast<HuffmanLeafNode>();

    if (code == 0 && bits == 0) {
      bits = 1;
    } else if (bits > BITS_IN_BYTE) {
      code = leaf_node.getByte();
      std::uint8_t empty_part = 0;
      for (auto i = BITS_IN_BYTE - 1; i > 0; i--)
        if ((code >> i) == 0)
          empty_part++;
      bits = BITS_IN_BYTE - empty_part;
    }

    table.insert(
        {leaf_node.getByte(),
         {leaf_node.getWeight(), static_cast<std::uint8_t>(code), bits}});
  } else {
    auto internal_node = node.downcast<HuffmanInternalNode>();

    code <<= 1;
    bits++;

    computePrefixCodePerByte(internal_node.getLeftNode(), table, code, bits);
    computePrefixCodePerByte(internal_node.getRightNode(), table, code | 1,
                             bits);
  }
}

std::unordered_map<std::uint8_t, PrefixCodeEntry>
computePrefixCodeTable(const std::shared_ptr<HuffmanTree> tree) {
  std::unordered_map<std::uint8_t, PrefixCodeEntry> table;

  computePrefixCodePerByte(tree->getRoot(), table);

  return table;
}

Output encodePrefixTableAndInput(
    const std::unordered_map<std::uint8_t, PrefixCodeEntry> &table,
    const std::vector<std::uint8_t> &in) {
  FlatBufferBuilder builder;
  std::size_t compressed_content_bits_sz = 0;

  std::vector<Offset<PrefixEntry>> prefix_entries;
  for (const auto &entry : table) {
    auto prefix_entry =
        CreatePrefixEntry(builder, static_cast<std::uint8_t>(entry.first),
                          entry.second.getFrequency());
    prefix_entries.push_back(prefix_entry);

    compressed_content_bits_sz +=
        entry.second.getFrequency() * entry.second.getBits();
  }
  auto entries = builder.CreateVector(prefix_entries);
  auto prefix_table = CreatePrefixTable(builder, entries);
  builder.Finish(prefix_table);

  std::uint32_t table_buff_sz = builder.GetSize();
  auto table_buff = builder.GetBufferPointer();

  auto compressed_in_sz = static_cast<std::uint32_t>(
      std::ceil(compressed_content_bits_sz / static_cast<float>(BITS_IN_BYTE)));

  std::uint32_t out_sz = SHA256_DIGEST_LENGTH + sizeof table_buff_sz +
                         sizeof compressed_in_sz + table_buff_sz +
                         compressed_in_sz;
  auto out_data = new std::uint8_t[out_sz];

  auto encoded_table_sz = out_data + SHA256_DIGEST_LENGTH;
  big_endian::put<std::uint32_t>(table_buff_sz, encoded_table_sz);

  auto encoded_compressed_in_sz = encoded_table_sz + sizeof table_buff_sz;
  big_endian::put<std::uint32_t>(compressed_in_sz, encoded_compressed_in_sz);

  auto encoded_table = encoded_compressed_in_sz + sizeof compressed_in_sz;
  std::memcpy(encoded_table, table_buff, table_buff_sz);

  auto compressed_in = encoded_table + table_buff_sz;
  std::uint32_t current_byte_index = 0;
  std::uint8_t byte_index = 0;
  for (auto byte : in) {
    auto prefix_entry = table.find(byte);
    auto bits = prefix_entry->second.getBits();
    assert(bits <= BITS_IN_BYTE &&
           "entry bits must be truncated to byte size in bits");
    auto code = prefix_entry->second.getCode();
    std::uint8_t free_byte_indexes = BITS_IN_BYTE - byte_index;

    if (bits <= free_byte_indexes) {
      if (byte_index == 0)
        compressed_in[current_byte_index] = code << (BITS_IN_BYTE - bits);
      else
        compressed_in[current_byte_index] |=
            code << (BITS_IN_BYTE - byte_index - bits);
      byte_index += bits;
    } else {
      std::uint8_t remaining = bits - free_byte_indexes;
      compressed_in[current_byte_index] |= code >> remaining;
      current_byte_index++;
      compressed_in[current_byte_index] = (code & ~(~0 << remaining))
                                          << (BITS_IN_BYTE - remaining);
      byte_index = remaining;
    }

    if (byte_index == BITS_IN_BYTE) {
      byte_index = 0;
      current_byte_index++;
    }
  }

  auto hash_digest = out_data;
  SHA256(out_data + SHA256_DIGEST_LENGTH, out_sz - SHA256_DIGEST_LENGTH,
         hash_digest);

  return Output(out_data, out_sz);
}

Output decodePrefixTableAndInput(const std::vector<std::uint8_t> &in) {
  Output out;

  if (in.empty())
    throw TreeCoderError("file is empty");

  // TODO: implement

  // WARN: handle case when byte code == bits, i.e., there's only one byte type.

  return out;
}

TreeCoder::TreeCoder() {}

TreeCoder::~TreeCoder() {}

Output TreeCoder::encode(const std::vector<std::uint8_t> &in) {
  auto frequencies = computeFrequencyTable(in);

  if (frequencies.empty())
    throw TreeCoderError("file is empty");

  auto tree = HuffmanTree::build(frequencies);

  auto table = computePrefixCodeTable(tree);

  return encodePrefixTableAndInput(table, in);
}

Output TreeCoder::decode(const std::vector<std::uint8_t> &in) {
  return decodePrefixTableAndInput(in);
}
} // namespace treecoder
