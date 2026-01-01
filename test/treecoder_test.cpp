#include <bitset>
#include <cstdint>
#include <cstring>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

#include "endian/big_endian.hpp"
#include "file.hpp"
#include "hash.hpp"
#include "prefixtable_generated.h"
#include "treecoder.hpp"

using namespace hash;
using namespace flatbuffers;
using namespace testing;
using namespace treecoder;
using namespace prefixtable;
using namespace endian;

TEST(FrequencyTableTest, EmptyTable) {
  Container in;

  auto frequencies = computeFrequencyTable(in);

  ASSERT_EQ(frequencies.size(), 0);
}

TEST(FrequencyTableTest, PopulatedTable) {
  Container in(reinterpret_cast<std::uint8_t *>(const_cast<char *>("aaba")), 4,
               false);

  auto frequencies = computeFrequencyTable(in);

  ASSERT_EQ(frequencies.size(), 2);

  auto a_freq = frequencies.find('a');
  ASSERT_NE(a_freq, frequencies.end());
  ASSERT_EQ(a_freq->second, 3);

  auto b_freq = frequencies.find('b');
  ASSERT_NE(a_freq, frequencies.end());
  ASSERT_EQ(b_freq->second, 1);
}

TEST(HuffmanTreeTest, PopulatedTree) {
  std::unordered_map<std::uint8_t, std::uint32_t> frequencies = {{'a', 3},
                                                                 {'b', 1}};

  auto tree = HuffmanTree::build(frequencies);

  ASSERT_EQ(tree->getRoot().getWeight(), 4);
  ASSERT_FALSE(tree->getRoot().isLeaf());

  auto root_node = tree->getRoot().downcast<HuffmanInternalNode>();
  ASSERT_TRUE(root_node.getLeftNode().isLeaf());
  ASSERT_TRUE(root_node.getRightNode().isLeaf());

  auto left_node = root_node.getLeftNode().downcast<HuffmanLeafNode>();
  ASSERT_EQ(left_node.getByte(), 'b');
  ASSERT_EQ(left_node.getWeight(), 1);

  auto right_node = root_node.getRightNode().downcast<HuffmanLeafNode>();
  ASSERT_EQ(right_node.getByte(), 'a');
  ASSERT_EQ(right_node.getWeight(), 3);
}

TEST(HuffmanTreeTest, OneEntryTree) {
  std::unordered_map<std::uint8_t, std::uint32_t> frequencies = {{'a', 3}};

  auto tree = HuffmanTree::build(frequencies);

  ASSERT_EQ(tree->getRoot().getWeight(), 3);
  ASSERT_TRUE(tree->getRoot().isLeaf());

  auto root_node = tree->getRoot().downcast<HuffmanLeafNode>();
  ASSERT_EQ(root_node.getByte(), 'a');
  ASSERT_EQ(root_node.getWeight(), 3);
}

TEST(HuffmanTreeTest, DoNotAcceptZeroFrequencies) {
  std::unordered_map<std::uint8_t, std::uint32_t> frequencies;

  ASSERT_DEATH(
      { HuffmanTree::build(frequencies); },
      "frequencies table must contain at least one entry");
}

struct ByteCode {
  std::uint8_t byte, bits;
};

class PrefixCodePerByteFixtureTest : public TestWithParam<ByteCode> {};

INSTANTIATE_TEST_SUITE_P(ByteCodes, PrefixCodePerByteFixtureTest,
                         Values(ByteCode{'a', 7}, ByteCode{'\0', 1},
                                ByteCode{0xFF, 8}, ByteCode{1, 1}));

TEST_P(PrefixCodePerByteFixtureTest, ByteCodeOverflows) {
  auto bc = GetParam();

  HuffmanLeafNode node(bc.byte, 12);
  std::unordered_map<std::uint8_t, PrefixCodeEntry> table;

  computePrefixCodePerByte(node, table, 0b1100110111, 10);

  auto entry = table.find(bc.byte);
  ASSERT_NE(entry, table.end());
  ASSERT_EQ(entry->second.getFrequency(), 12);
  ASSERT_EQ(entry->second.getCode(), bc.byte);
  ASSERT_EQ(entry->second.getBits(), bc.bits);
}

TEST(PrefixCodePerByteTest, OnlyOneByteCode) {
  HuffmanLeafNode node('a', 12);
  std::unordered_map<std::uint8_t, PrefixCodeEntry> table;

  computePrefixCodePerByte(node, table);

  auto entry = table.find('a');
  ASSERT_NE(entry, table.end());
  ASSERT_EQ(entry->second.getFrequency(), 12);
  ASSERT_EQ(entry->second.getCode(), 0);
  ASSERT_EQ(entry->second.getBits(), 1);
}

TEST(PrefixCodeTableTest, PopulatedTable) {
#define JOIN(L, R) std::make_shared<HuffmanTree>(L, R)
#define LEAF(B, W) std::make_shared<HuffmanTree>(B, W)

  std::shared_ptr<HuffmanTree> tree =
      JOIN(LEAF('E', 120),
           JOIN(JOIN(LEAF('U', 37), LEAF('D', 42)),
                JOIN(LEAF('L', 42),
                     JOIN(LEAF('C', 32), JOIN(JOIN(LEAF('Z', 2), LEAF('K', 7)),
                                              LEAF('M', 24))))));

#undef JOIN
#undef LEAF

  auto table = computePrefixCodeTable(tree);

#define MATCHES(B, EF, EC, EB)                                                 \
  {                                                                            \
    auto entry = table.find(B);                                                \
    ASSERT_NE(entry, table.end());                                             \
    auto got = entry->second;                                                  \
    ASSERT_EQ(got.getFrequency(), EF);                                         \
    ASSERT_EQ(got.getCode(), EC);                                              \
    ASSERT_EQ(got.getBits(), EB);                                              \
  }

  MATCHES('C', 32, 0b1110, 4);
  MATCHES('D', 42, 0b101, 3);
  MATCHES('E', 120, 0b0, 1);
  MATCHES('K', 7, 0b111101, 6);
  MATCHES('L', 42, 0b110, 3);
  MATCHES('M', 24, 0b11111, 5);
  MATCHES('U', 37, 0b100, 3);
  MATCHES('Z', 2, 0b111100, 6);

#undef MATCHES
}

TEST(PrefixCodeTableTest, OneTableEntry) {
  std::shared_ptr<HuffmanTree> tree = std::make_shared<HuffmanTree>('C', 69);

  auto table = computePrefixCodeTable(tree);

  auto entry = table.find('C');
  ASSERT_NE(entry, table.end());

  auto got = entry->second;
  ASSERT_EQ(got.getFrequency(), 69);
  ASSERT_EQ(got.getCode(), 0);
  ASSERT_EQ(got.getBits(), 1);
}

TEST(EncodePrefixTableAndInputTest, PopulatedTable) {
  std::unordered_map<std::uint8_t, PrefixCodeEntry> table = {
      {'A', {3, 0b110, 3}}, {'B', {2, 0b010, 3}}, {'C', {4, 0b00, 2}},
      {'D', {6, 0b10, 2}},  {'E', {2, 0b011, 3}}, {'F', {3, 0b111, 3}}};

  Container in(reinterpret_cast<std::uint8_t *>(
                   const_cast<char *>("AAABCCBCCDDDEEFDDFDF")),
               20, false);

  std::vector<std::uint8_t> expected_compressed_out = {
      0b11011011, 0b00100000, 0b01000001, 0b01010011,
      0b01111110, 0b10111101, 0b11000000};

  std::uint8_t expected_out_hash[HASH_DIGEST_LENGTH];

  auto out = encodePrefixTableAndInput(table, in);

  std::uint32_t encoded_table_sz;
  std::uint32_t encoded_compressed_in_sz;

  ASSERT_TRUE(out.getSize() >= sizeof expected_out_hash +
                                   sizeof encoded_table_sz +
                                   sizeof encoded_compressed_in_sz);

  big_endian::get<std::uint32_t>(encoded_table_sz,
                                 out.getData() + sizeof expected_out_hash);

  big_endian::get<std::uint32_t>(
      encoded_compressed_in_sz, out.getData() + sizeof expected_out_hash +
                                    sizeof encoded_table_sz + encoded_table_sz);

  ASSERT_TRUE(out.getSize() - sizeof expected_out_hash -
                  sizeof encoded_table_sz - sizeof encoded_compressed_in_sz ==
              encoded_table_sz + expected_compressed_out.size());

  computeHash(out.getData() + sizeof expected_out_hash,
              out.getSize() - sizeof expected_out_hash, expected_out_hash);
  ASSERT_EQ(
      std::memcmp(out.getData(), expected_out_hash, sizeof expected_out_hash),
      0);
  ;

  Verifier verifier(out.getData() + sizeof expected_out_hash +
                        sizeof encoded_table_sz,
                    out.getSize());
  ASSERT_TRUE(VerifyPrefixTableBuffer(verifier));

  auto prefix_table = GetPrefixTable(out.getData() + sizeof expected_out_hash +
                                     sizeof encoded_table_sz);
  ASSERT_EQ(prefix_table->entries()->size(), table.size());
  for (auto prefix_entry : *prefix_table->entries()) {
    auto entry = table.find(prefix_entry->byte());
    ASSERT_NE(entry, table.end());
    ASSERT_EQ(prefix_entry->frequency(), entry->second.getFrequency());
  }

  for (auto i = 0; i < expected_compressed_out.size(); i++) {
    auto got_byte = std::bitset<std::numeric_limits<std::uint8_t>::digits>{
        out.getData()[i + sizeof expected_out_hash + sizeof encoded_table_sz +
                      encoded_table_sz + sizeof encoded_compressed_in_sz]};
    auto expected_byte = std::bitset<std::numeric_limits<std::uint8_t>::digits>{
        expected_compressed_out[i]};
    ASSERT_EQ(got_byte, expected_byte);
  }
}

TEST(EncodePrefixTableAndInputTest, DoNotAcceptCodeWithMoreThan8Bits) {
  std::unordered_map<std::uint8_t, PrefixCodeEntry> table = {
      {'A', {3, 0b110, 9}}};

  Container in(reinterpret_cast<std::uint8_t *>(const_cast<char *>("AAA")), 3,
               false);

  ASSERT_DEATH(
      { encodePrefixTableAndInput(table, in); },
      "entry bits must be truncated to byte size in bits");
}

TEST(EncodeTest, EmptyInput) {
  TreeCoder tc;
  Container in;

  ASSERT_THROW({ tc.encode(in); }, TreeCoderError);
}

TEST(TryLocateSectionsTest, ValidSections) {
  TreeCoder tc;

  Container in(reinterpret_cast<std::uint8_t *>(const_cast<char *>("AAA")), 3,
               false);

  auto out = tc.encode(in);

  auto possible_sections = tryLocateSections(out);
  ASSERT_TRUE(possible_sections.has_value());
  auto sections = possible_sections.value();

  Verifier verifier(sections.getEncodedTable(), sections.getEncodedTableSize());
  ASSERT_TRUE(VerifyPrefixTableBuffer(verifier));

  ASSERT_EQ(sections.getEncodedCompressedInSize(), 1);
  ASSERT_EQ(sections.getEncodedCompressedInput()[0], 0);
}

TEST(TryLocateSectionsTest, TableSizeDoNotExist) {
  TreeCoder tc;

  Container in(reinterpret_cast<std::uint8_t *>(const_cast<char *>("AAA")), 3,
               false);

  auto out = tc.encode(in);

  Container _out(const_cast<std::uint8_t *>(out.getData()), 4, false);
  ASSERT_FALSE(tryLocateSections(_out).has_value());
}

TEST(TryLocateSectionsTest, TableSectionIsInvalid) {
  TreeCoder tc;

  Container in(reinterpret_cast<std::uint8_t *>(const_cast<char *>("AAA")), 3,
               false);

  auto out = tc.encode(in);

  Container _out(const_cast<std::uint8_t *>(out.getData()),
                 HASH_DIGEST_LENGTH + sizeof(std::uint32_t) + 4, false);
  ASSERT_FALSE(tryLocateSections(_out).has_value());
}

TEST(TryLocateSectionsTest, CompressedSectionIsInvalid) {
  TreeCoder tc;

  Container in(reinterpret_cast<std::uint8_t *>(const_cast<char *>("AAA")), 3,
               false);

  auto out = tc.encode(in);

  Container _out(const_cast<std::uint8_t *>(out.getData()), out.getSize() - 1,
                 false);
  ASSERT_FALSE(tryLocateSections(_out).has_value());
}
