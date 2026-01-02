#pragma once

#include <cstdint>
#include <memory>

namespace container {
class Container {
private:
  std::shared_ptr<std::uint8_t> data;
  std::uint32_t size;

public:
  Container();

  Container(std::uint32_t size);

  ~Container();

  std::uint8_t *getData() const;

  std::uint32_t getSize() const;

  std::uint32_t isEmpty() const;
};
} // namespace container
