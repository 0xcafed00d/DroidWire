#ifndef DROID_WIRE_BUFFER__H
#define DROID_WIRE_BUFFER__H

#include <cstddef>
#include <cstdint>
#include <span>

namespace DroidWire {

	struct BufferInterface {
		virtual ~BufferInterface() = default;

		virtual size_t capacity() = 0;
		virtual size_t size() const = 0;
		virtual std::span<std::byte> data() = 0;
		virtual std::span<const std::byte> data() const = 0;

		virtual void append(const std::span<const std::byte>& bytes) = 0;
		virtual void consume(size_t count) = 0;
	};

}  // namespace DroidWire

#endif