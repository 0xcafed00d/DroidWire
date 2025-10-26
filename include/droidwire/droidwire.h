#ifndef DROIDWIRE_H
#define DROIDWIRE_H

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <variant>

namespace DroidWire {

	enum class Parity { None, Even, Odd };
	enum class StopBits { One, Two };
	enum class FlowControl { None, Hardware, Software };

	enum class BaudRate {
		B110,
		B300,
		B600,
		B1200,
		B2400,
		B4800,
		B9600,
		B14400,
		B19200,
		B38400,
		B57600,
		B115200,
		B128000,
		B230400,
		B256000,
		B460800,
		B921600
	};

	// Either a standard baud enum or a custom integer
	using BaudSetting = std::variant<BaudRate, unsigned int>;

	struct SerialConfig {
		std::string device;
		BaudSetting baudRate = BaudRate::B115200;
		Parity parity = Parity::None;
		StopBits stopBits = StopBits::One;
		FlowControl flowControl = FlowControl::None;
		std::chrono::milliseconds timeout = std::chrono::milliseconds{1000};
		bool async = false;
	};

	class SerialPort {
	   public:
		explicit SerialPort(const SerialConfig& config);
		~SerialPort();

		SerialPort(const SerialPort&) = delete;
		SerialPort& operator=(const SerialPort&) = delete;
		SerialPort(SerialPort&&) noexcept;
		SerialPort& operator=(SerialPort&&) noexcept;

		std::span<const std::byte> write(std::span<const std::byte> data);
		std::span<std::byte> read(std::span<std::byte> buffer);

		std::future<std::span<const std::byte>> writeAsync(std::span<const std::byte> data);
		std::future<std::span<std::byte>> readAsync(std::span<std::byte> buffer);

	   private:
		bool async_ = false;

#ifdef _WIN32
		void* handle_ = nullptr;  // HANDLE
#else
		int fd_ = -1;
#endif
	};

	// --- Utility: convert baud settings to human-readable strings ---
	std::string to_string(BaudRate rate);
	std::string to_string(const BaudSetting& setting);
}  // namespace DroidWire

#endif  // DROIDWIRE_H