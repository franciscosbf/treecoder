#include "container.hpp"

namespace container {
Container::Container() : data(nullptr), size(0) {}

void doNotDelete(std::uint8_t *data) {}

Container::Container(std::uint32_t size)
    : data(new std::uint8_t[size]), size(size) {}

Container::~Container() {}

std::uint8_t *Container::getData() const { return data.get(); }

std::uint32_t Container::getSize() const { return size; }

std::uint32_t Container::isEmpty() const { return size == 0; }
} // namespace container
