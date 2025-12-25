#ifndef DROID_WIRE_BUFFER__H
#define DROID_WIRE_BUFFER__H

#include <array>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <vector>

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

	class DynamicBuffer final : public BufferInterface {
	   public:
		// initial_capacity reserves storage without changing the buffer size.
		explicit DynamicBuffer(size_t initial_capacity = 0);

		size_t capacity() override;
		size_t size() const override;
		std::span<std::byte> data() override;
		std::span<const std::byte> data() const override;

		void append(const std::span<const std::byte>& bytes) override;
		void consume(size_t count) override;

	   private:
		void ensure_capacity(size_t needed);

		std::vector<std::byte> m_storage;
		size_t m_size = 0;
	};

	template <size_t N>
	class FixedBuffer final : public BufferInterface {
	   public:
		size_t capacity() override {
			return N;
		}
		size_t size() const override {
			return m_size;
		}
		std::span<std::byte> data() override {
			return {m_storage.data(), m_size};
		}
		std::span<const std::byte> data() const override {
			return {m_storage.data(), m_size};
		}

		void append(const std::span<const std::byte>& bytes) override {
			if (bytes.empty())
				return;
			if (m_size + bytes.size() > N)
				throw std::out_of_range("append exceeds fixed buffer capacity");
			std::copy(bytes.begin(), bytes.end(), m_storage.data() + m_size);
			m_size += bytes.size();
		}

		void consume(size_t count) override {
			if (count > m_size)
				throw std::out_of_range("consume exceeds buffer size");
			if (count == 0)
				return;
			std::rotate(m_storage.begin(), m_storage.begin() + count, m_storage.begin() + m_size);
			m_size -= count;
		}

	   private:
		std::array<std::byte, N> m_storage{};
		size_t m_size = 0;
	};

}  // namespace DroidWire

#endif
