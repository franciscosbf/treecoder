#pragma once

#include <cstdint>

#include "openssl/sha.h"

namespace hash {
const std::uint32_t HASH_DIGEST_LENGTH = SHA256_DIGEST_LENGTH;

void computeHash(const std::uint8_t *input, std::uint32_t input_sz,
                 std::uint8_t (&digest)[HASH_DIGEST_LENGTH]);
} // namespace hash
