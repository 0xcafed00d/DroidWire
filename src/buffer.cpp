#include "droidwire/buffer.h"

#include <algorithm>

namespace DroidWire {

	DynamicBuffer::DynamicBuffer(size_t initial_capacity) : m_storage(), m_size(0) {
		m_storage.reserve(initial_capacity);
	}

	size_t DynamicBuffer::capacity() {
		return m_storage.capacity();
	}

	size_t DynamicBuffer::size() const {
		return m_size;
	}

	std::span<std::byte> DynamicBuffer::data() {
		return {m_storage.data(), m_size};
	}

	std::span<const std::byte> DynamicBuffer::data() const {
		return {m_storage.data(), m_size};
	}

	void DynamicBuffer::append(const std::span<const std::byte>& bytes) {
		if (bytes.empty())
			return;
		ensure_capacity(m_size + bytes.size());
		std::copy(bytes.begin(), bytes.end(), m_storage.data() + m_size);
		m_size += bytes.size();
	}

	void DynamicBuffer::consume(size_t count) {
		if (count > m_size)
			throw std::out_of_range("consume exceeds buffer size");
		if (count == 0)
			return;
		std::rotate(m_storage.begin(), m_storage.begin() + count, m_storage.begin() + m_size);
		m_size -= count;
	}

	void DynamicBuffer::ensure_capacity(size_t needed) {
		if (needed <= m_storage.size())
			return;
		m_storage.resize(needed);
	}

}  // namespace DroidWire
