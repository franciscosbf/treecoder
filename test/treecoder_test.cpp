#include <bitset>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "container.hpp"
#include "endian/big_endian.hpp"
#include "hash.hpp"
#include "prefixtable_generated.h"
#include "treecoder.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace hash;
using namespace container;
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
  Container in(4);
  std::memcpy(in.getData(), "aaba", in.getSize());

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
  std::vector<std::pair<std::uint8_t, std::uint32_t>> frequencies = {{'a', 3},
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
  std::vector<std::pair<std::uint8_t, std::uint32_t>> frequencies = {{'a', 3}};

  auto tree = HuffmanTree::build(frequencies);

  ASSERT_EQ(tree->getRoot().getWeight(), 3);
  ASSERT_TRUE(tree->getRoot().isLeaf());

  auto root_node = tree->getRoot().downcast<HuffmanLeafNode>();
  ASSERT_EQ(root_node.getByte(), 'a');
  ASSERT_EQ(root_node.getWeight(), 3);
}

TEST(HuffmanTreeTest, DoNotAcceptZeroFrequencies) {
  std::vector<std::pair<std::uint8_t, std::uint32_t>> frequencies;

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
  std::vector<std::pair<std::uint8_t, std::uint32_t>> static_frequencies = {
      {'A', 3}, {'B', 2}, {'C', 4}, {'D', 6}, {'E', 2}, {'F', 3}};

  Container in(20);
  std::memcpy(in.getData(), "AAABCCBCCDDDEEFDDFDF", in.getSize());

  std::vector<std::uint8_t> expected_compressed_out = {
      0b11011011, 0b00100000, 0b01000001, 0b01010011,
      0b01111110, 0b10111101, 0b11000000};

  std::uint8_t expected_out_hash[HASH_DIGEST_LENGTH];

  auto out = encodePrefixTableAndInput(static_frequencies, table, in);

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
  std::vector<std::pair<std::uint8_t, std::uint32_t>> static_frequencies = {
      {'A', 3}};

  Container in(3);
  std::memcpy(in.getData(), "AAA", in.getSize());

  ASSERT_DEATH(
      { encodePrefixTableAndInput(static_frequencies, table, in); },
      "entry bits must be truncated to byte size in bits");
}

TEST(IsInputUntampered, ValidInput) {
  TreeCoder tc;

  Container in(3);
  std::memcpy(in.getData(), "AAA", in.getSize());

  auto out = tc.encode(in);

  ASSERT_TRUE(isInputUntampered(out));
}

TEST(IsInputUntampered, InputIsSmallerThanHashSize) {
  TreeCoder tc;

  Container in(3);
  std::memcpy(in.getData(), "AAA", in.getSize());

  auto out = tc.encode(in);

  Container _out(HASH_DIGEST_LENGTH - 3);
  std::memcpy(_out.getData(), out.getData(), _out.getSize());
  ASSERT_FALSE(isInputUntampered(_out));
}

TEST(IsInputUntampered, HashWasForged) {
  TreeCoder tc;

  Container in(3);
  std::memcpy(in.getData(), "AAA", in.getSize());

  auto out = tc.encode(in);

  out.getData()[1] = 3;
  ASSERT_FALSE(isInputUntampered(out));
}

TEST(IsInputUntampered, EncodedPartWasForged) {
  TreeCoder tc;

  Container in(3);
  std::memcpy(in.getData(), "AAA", in.getSize());

  auto out = tc.encode(in);

  out.getData()[HASH_DIGEST_LENGTH + sizeof(std::uint32_t)] = 3;
  ASSERT_FALSE(isInputUntampered(out));
}

TEST(EncodeTest, EmptyInput) {
  TreeCoder tc;
  Container in;

  ASSERT_THROW({ tc.encode(in); }, TreeCoderError);
}

TEST(TryLocateSectionsTest, ValidSections) {
  TreeCoder tc;

  Container in(3);
  std::memcpy(in.getData(), "AAA", in.getSize());

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

  Container in(3);
  std::memcpy(in.getData(), "AAA", in.getSize());

  auto out = tc.encode(in);

  Container _out(4);
  std::memcpy(_out.getData(), out.getData(), _out.getSize());
  ASSERT_FALSE(tryLocateSections(_out).has_value());
}

TEST(TryLocateSectionsTest, TableSectionIsInvalid) {
  TreeCoder tc;

  Container in(3);
  std::memcpy(in.getData(), "AAA", in.getSize());

  auto out = tc.encode(in);

  Container _out(HASH_DIGEST_LENGTH + sizeof(std::uint32_t) + 4);
  std::memcpy(_out.getData(), out.getData(), _out.getSize());
  ASSERT_FALSE(tryLocateSections(_out).has_value());
}

TEST(TryLocateSectionsTest, CompressedSectionIsInvalid) {
  TreeCoder tc;

  Container in(3);
  std::memcpy(in.getData(), "AAA", in.getSize());

  auto out = tc.encode(in);

  Container _out(out.getSize() - 1);
  std::memcpy(_out.getData(), out.getData(), _out.getSize());
  ASSERT_FALSE(tryLocateSections(_out).has_value());
}

TEST(TryDecodePrefixTableTest, ValidTable) {
  FlatBufferBuilder builder;
  auto entries = builder.CreateVector({CreatePrefixEntry(builder, 'A', 2)});
  auto prefix_table = CreatePrefixTable(builder, entries);
  builder.Finish(prefix_table);

  auto possible_table =
      tryDecodePrefixTable(builder.GetBufferPointer(), builder.GetSize());
  ASSERT_TRUE(possible_table.has_value());
  auto table = possible_table.value();

  ASSERT_EQ(table.size(), 1);
  auto frequency = table[0];
  ASSERT_EQ(frequency.first, 'A');
  ASSERT_EQ(frequency.second, 2);
}

TEST(TryDecodePrefixTableTest, InvalidTable) {
  FlatBufferBuilder builder;
  auto entries = builder.CreateVector({CreatePrefixEntry(builder, 'A', 2)});
  auto prefix_table = CreatePrefixTable(builder, entries);
  builder.Finish(prefix_table);

  auto possible_table =
      tryDecodePrefixTable(builder.GetBufferPointer(), builder.GetSize() - 4);
  ASSERT_FALSE(possible_table.has_value());
}

TEST(CalcNumberOfCompressedBytes, PopulatedTable) {
  std::vector<std::pair<std::uint8_t, std::uint32_t>> table = {{'A', 3},
                                                               {'B', 4}};

  ASSERT_EQ(calcNumberOfCompressedBytes(table), 7);
}

TEST(CalcNumberOfCompressedBytes, EmptyTable) {
  std::vector<std::pair<std::uint8_t, std::uint32_t>> table;

  ASSERT_EQ(calcNumberOfCompressedBytes(table), 0);
}

TEST(TryFindByteTest, ValidCompression) {
  std::uint8_t compressed[] = {0b01010111, 0b10000000};

#define JOIN(L, R) std::make_shared<HuffmanTree>(L, R)
#define LEAF(B, W) std::make_shared<HuffmanTree>(B, W)

  std::shared_ptr<HuffmanTree> tree =
      JOIN(LEAF('C', 1), JOIN(LEAF('B', 2), LEAF('A', 2)));

#undef JOIN
#undef LEAF

  auto possible_out = tryDecodeInput(5, tree, compressed, 2);
  ASSERT_TRUE(possible_out.has_value());
  auto out = possible_out.value();

  auto got_decoded =
      std::string(reinterpret_cast<char *>(out.getData()), out.getSize());
  ASSERT_THAT(got_decoded, StrEq("CBBAA"));
}

TEST(TryFindByteTest, ValidCompressionWithOnlyOneByteType) {
  std::uint8_t compressed[] = {0b01010111};

#define JOIN(L, R) std::make_shared<HuffmanTree>(L, R)
#define LEAF(B, W) std::make_shared<HuffmanTree>(B, W)

  std::shared_ptr<HuffmanTree> tree = LEAF('C', 3);

#undef JOIN
#undef LEAF

  auto possible_out = tryDecodeInput(3, tree, compressed, 1);
  ASSERT_TRUE(possible_out.has_value());
  auto out = possible_out.value();

  auto got_decoded =
      std::string(reinterpret_cast<char *>(out.getData()), out.getSize());
  ASSERT_THAT(got_decoded, StrEq("CCC"));
}

TEST(DecodeTest, ValidCompressedInput) {
  TreeCoder tc;

  auto expected_uncompressed = "ABBC DD @@";
  Container in(10);
  std::memcpy(in.getData(), expected_uncompressed, in.getSize());

  Container out;
  {
    Container compressed = tc.encode(in);
    out = tc.decode(compressed);
  }

  auto got_uncompressed =
      std::string(reinterpret_cast<char *>(out.getData()), out.getSize());
  ASSERT_THAT(got_uncompressed, expected_uncompressed);
}
