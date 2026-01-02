#include <cassert>
#include <cmath>
#include <cstring>
#include <memory>
#include <queue>

#include "container.hpp"
#include "endian/big_endian.hpp"
#include "hash.hpp"
#include "prefixtable_generated.h"
#include "treecoder.hpp"

using namespace hash;
using namespace endian;
using namespace flatbuffers;
using namespace prefixtable;

namespace treecoder {
constexpr std::uint32_t N_BYTE_BITS = std::numeric_limits<std::uint8_t>::digits;

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

std::shared_ptr<HuffmanTree>
HuffmanTree::build(const std::vector<std::pair<std::uint8_t, std::uint32_t>>
                       &static_frequencies) {
  assert(!static_frequencies.empty() &&
         "frequencies table must contain at least one entry");

  std::priority_queue<std::shared_ptr<HuffmanTree>,
                      std::vector<std::shared_ptr<HuffmanTree>>,
                      ComparableHuffmanTree>
      prioritized_frequencies;

  for (const auto &frequency : static_frequencies)
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

std::unordered_map<std::uint8_t, std::uint32_t>
computeFrequencyTable(const Container &in) {
  std::unordered_map<std::uint8_t, std::uint32_t> frequencies;

  for (auto i = 0; i < in.getSize(); i++) {
    if (auto frequency = frequencies.find(in.getData()[i]);
        frequency != frequencies.end())
      frequency->second++;
    else
      frequencies.insert({in.getData()[i], 1});
  }

  return frequencies;
}

const std::vector<std::pair<std::uint8_t, std::uint32_t>>
createStaticFrequencyTable(
    const std::unordered_map<std::uint8_t, std::uint32_t> &frequencies) {
  std::vector<std::pair<std::uint8_t, std::uint32_t>> static_frequencies;
  static_frequencies.reserve(frequencies.size());

  for (const auto &frequency : frequencies)
    static_frequencies.push_back({frequency.first, frequency.second});

  return static_frequencies;
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
    } else if (bits > N_BYTE_BITS) {
      code = leaf_node.getByte();
      std::uint8_t empty_part = 0;
      for (auto i = N_BYTE_BITS - 1; i > 0; i--)
        if ((code >> i) == 0)
          empty_part++;
      bits = N_BYTE_BITS - empty_part;
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

Container encodePrefixTableAndInput(
    const std::vector<std::pair<std::uint8_t, std::uint32_t>>
        &static_frequencies,
    const std::unordered_map<std::uint8_t, PrefixCodeEntry> &table,
    const Container &in) {
  FlatBufferBuilder builder;
  std::size_t compressed_content_bits_sz = 0;

  std::vector<Offset<PrefixEntry>> prefix_entries;
  for (const auto &frequency : static_frequencies) {
    auto prefix_entry =
        CreatePrefixEntry(builder, frequency.first, frequency.second);
    prefix_entries.push_back(prefix_entry);

    auto bits = table.find(frequency.first)->second.getBits();
    compressed_content_bits_sz += frequency.second * bits;
  }
  auto entries = builder.CreateVector(prefix_entries);
  auto prefix_table = CreatePrefixTable(builder, entries);
  builder.Finish(prefix_table);

  std::uint32_t table_buff_sz = builder.GetSize();
  auto table_buff = builder.GetBufferPointer();

  auto compressed_in_sz = static_cast<std::uint32_t>(
      std::ceil(compressed_content_bits_sz / static_cast<float>(N_BYTE_BITS)));

  std::uint32_t out_sz = HASH_DIGEST_LENGTH + sizeof table_buff_sz +
                         sizeof compressed_in_sz + table_buff_sz +
                         compressed_in_sz;
  Container out(out_sz);
  auto out_data = out.getData();

  auto encoded_table_sz = out_data + HASH_DIGEST_LENGTH;
  big_endian::put<std::uint32_t>(table_buff_sz, encoded_table_sz);

  auto encoded_table = encoded_table_sz + sizeof table_buff_sz;
  std::memcpy(encoded_table, table_buff, table_buff_sz);

  auto encoded_compressed_in_sz = encoded_table + table_buff_sz;
  big_endian::put<std::uint32_t>(compressed_in_sz, encoded_compressed_in_sz);

  auto compressed_in = encoded_compressed_in_sz + sizeof compressed_in_sz;
  std::uint32_t current_byte_index = 0;
  std::uint8_t byte_index = 0;
  for (auto i = 0; i < in.getSize(); i++) {
    auto prefix_entry = table.find(in.getData()[i]);
    auto bits = prefix_entry->second.getBits();
    assert(bits <= N_BYTE_BITS &&
           "entry bits must be truncated to byte size in bits");
    auto code = prefix_entry->second.getCode();
    std::uint8_t free_byte_indexes = N_BYTE_BITS - byte_index;

    if (bits <= free_byte_indexes) {
      if (byte_index == 0)
        compressed_in[current_byte_index] = code << (N_BYTE_BITS - bits);
      else
        compressed_in[current_byte_index] |=
            code << (N_BYTE_BITS - byte_index - bits);
      byte_index += bits;
    } else {
      std::uint8_t remaining = bits - free_byte_indexes;
      compressed_in[current_byte_index] |= code >> remaining;
      current_byte_index++;
      compressed_in[current_byte_index] = (code & ~(~0 << remaining))
                                          << (N_BYTE_BITS - remaining);
      byte_index = remaining;
    }

    if (byte_index == N_BYTE_BITS) {
      byte_index = 0;
      current_byte_index++;
    }
  }

  auto hash_digest = out_data;
  computeHash(
      out_data + HASH_DIGEST_LENGTH, out_sz - HASH_DIGEST_LENGTH,
      reinterpret_cast<std::uint8_t (&)[HASH_DIGEST_LENGTH]>(*hash_digest));

  return out;
}

bool isInputUntampered(const Container &in) {
  auto data = in.getData();
  auto size = in.getSize();

  if (size < HASH_DIGEST_LENGTH)
    return false;

  auto expected_hash_digest = data;
  std::uint8_t got_hash_digest[HASH_DIGEST_LENGTH];
  computeHash(data + HASH_DIGEST_LENGTH, size - HASH_DIGEST_LENGTH,
              got_hash_digest);

  return std::memcmp(got_hash_digest, expected_hash_digest,
                     HASH_DIGEST_LENGTH) == 0;
}

EncodedSections::EncodedSections(const std::uint8_t *encoded_table,
                                 std::uint32_t encoded_table_sz,
                                 const std::uint8_t *encoded_compressed_in,
                                 std::uint32_t encoded_compressed_in_sz)
    : encoded_table(encoded_table), table_sz(encoded_table_sz),
      encoded_compressed_in(encoded_compressed_in),
      compressed_in_sz(encoded_compressed_in_sz) {}

EncodedSections::~EncodedSections() {}

const std::uint8_t *EncodedSections::getEncodedTable() const {
  return encoded_table;
}

std::uint32_t EncodedSections::getEncodedTableSize() const { return table_sz; }

const std::uint8_t *EncodedSections::getEncodedCompressedInput() const {
  return encoded_compressed_in;
}

std::uint32_t EncodedSections::getEncodedCompressedInSize() const {
  return compressed_in_sz;
}

std::optional<EncodedSections> tryLocateSections(const Container &in) {
  auto data = in.getData();
  auto size = in.getSize();

  if (size < HASH_DIGEST_LENGTH + sizeof(std::uint32_t))
    return {};

  std::uint32_t table_sz;
  auto encoded_table_sz = data + HASH_DIGEST_LENGTH;
  big_endian::get(table_sz, encoded_table_sz);
  auto encoded_table = encoded_table_sz + sizeof(std::uint32_t);

  if (size < (encoded_table + table_sz) - encoded_table)
    return {};

  std::uint32_t compressed_in_sz;
  auto encoded_compressed_in_sz = encoded_table + table_sz;
  big_endian::get(compressed_in_sz, encoded_compressed_in_sz);
  auto encoded_compressed_in = encoded_compressed_in_sz + sizeof(std::uint32_t);

  if (size != (encoded_compressed_in + compressed_in_sz) - in.getData())
    return {};

  return EncodedSections(encoded_table, table_sz, encoded_compressed_in,
                         compressed_in_sz);
}

std::optional<std::vector<std::pair<std::uint8_t, std::uint32_t>>>
tryDecodePrefixTable(const std::uint8_t *encoded_table,
                     std::uint32_t encoded_table_sz) {
  std::vector<std::pair<std::uint8_t, std::uint32_t>> table;

  Verifier verifier(encoded_table, encoded_table_sz);
  if (!VerifyPrefixTableBuffer(verifier))
    return {};

  auto prefix_table = GetPrefixTable(encoded_table);
  for (const auto &prefix_entry : *prefix_table->entries())
    table.push_back({prefix_entry->byte(), prefix_entry->frequency()});

  return table;
}

std::uint32_t calcNumberOfCompressedBytes(
    const std::vector<std::pair<std::uint8_t, std::uint32_t>> &table) {
  std::uint32_t compressed_bytes = 0;

  for (auto frequency : table)
    compressed_bytes += frequency.second;

  return compressed_bytes;
}

std::optional<std::uint8_t>
tryFindByte(const HuffmanNode &node, const std::uint8_t *encoded_compressed_in,
            std::uint32_t encoded_compressed_in_sz,
            std::uint32_t compressed_bytes, std::uint32_t &current_byte,
            std::uint8_t &current_byte_index) {
  if (current_byte == encoded_compressed_in_sz)
    return {};
  else if (node.isLeaf())
    return node.downcast<HuffmanLeafNode>().getByte();
  else {
    auto bit = (encoded_compressed_in[current_byte] >> current_byte_index) & 1;
    if (current_byte_index == 0) {
      current_byte_index = N_BYTE_BITS - 1;
      current_byte++;
    } else
      current_byte_index--;
    auto internal_node = node.downcast<HuffmanInternalNode>();
    return tryFindByte(bit == 1 ? internal_node.getRightNode()
                                : internal_node.getLeftNode(),
                       encoded_compressed_in, encoded_compressed_in_sz,
                       compressed_bytes, current_byte, current_byte_index);
  }
}

std::optional<Container>
tryDecodeInput(std::uint32_t compressed_bytes,
               const std::shared_ptr<HuffmanTree> tree,
               const std::uint8_t *encoded_compressed_in,
               std::uint32_t encoded_compressed_in_sz) {
  Container out(compressed_bytes);
  auto out_data = out.getData();
  const HuffmanNode &root = tree->getRoot();
  std::uint32_t current_byte = 0;
  std::uint8_t current_byte_index = N_BYTE_BITS - 1;

  for (auto i = 0; i < compressed_bytes; i++) {
    auto possible_byte =
        tryFindByte(root, encoded_compressed_in, encoded_compressed_in_sz,
                    compressed_bytes, current_byte, current_byte_index);
    if (!possible_byte.has_value())
      return {};
    out_data[i] = possible_byte.value();
  }

  return out;
}

TreeCoder::TreeCoder() {}

TreeCoder::~TreeCoder() {}

Container TreeCoder::encode(const Container &in) {
  auto frequencies = computeFrequencyTable(in);
  if (frequencies.empty())
    throw TreeCoderError("file is empty");

  auto static_frequencies = createStaticFrequencyTable(frequencies);

  auto tree = HuffmanTree::build(static_frequencies);

  auto table = computePrefixCodeTable(tree);

  return encodePrefixTableAndInput(static_frequencies, table, in);
}

Container TreeCoder::decode(const Container &in) {
  if (in.isEmpty())
    throw TreeCoderError("file is empty");

  if (!isInputUntampered(in))
    throw TreeCoderError("encoded file was altered");

  auto possible_encoded_sections = tryLocateSections(in);
  if (!possible_encoded_sections.has_value())
    throw TreeCoderError("unable to locate encoded file content sections");
  auto encoded_sections = possible_encoded_sections.value();

  auto possible_table =
      tryDecodePrefixTable(encoded_sections.getEncodedTable(),
                           encoded_sections.getEncodedTableSize());
  if (!possible_table.has_value())
    throw TreeCoderError(
        "file contains invalid data in prefix code table section");
  auto table = possible_table.value();
  if (table.empty())
    throw TreeCoderError("file contains empty prefix code table");

  auto compressed_bytes = calcNumberOfCompressedBytes(table);
  auto tree = HuffmanTree::build(table);

  auto possible_out = tryDecodeInput(
      compressed_bytes, tree, encoded_sections.getEncodedCompressedInput(),
      encoded_sections.getEncodedCompressedInSize());
  if (!possible_out.has_value())
    throw TreeCoderError("file contains invalid data in compressed section or "
                         "prefix code table section");
  auto out = possible_out.value();

  return out;
}
} // namespace treecoder
