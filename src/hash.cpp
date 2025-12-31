#include "hash.hpp"

namespace hash {
void computeHash(const std::uint8_t *input, std::uint32_t input_sz,
                 std::uint8_t (&digest)[HASH_DIGEST_LENGTH]) {
  SHA256(input, input_sz, digest);
}
} // namespace hash
