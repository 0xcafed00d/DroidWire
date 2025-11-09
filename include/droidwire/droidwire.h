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
		Baud110,
		Baud300,
		Baud600,
		Baud1200,
		Baud2400,
		Baud4800,
		Baud9600,
		Baud14400,
		Baud19200,
		Baud38400,
		Baud57600,
		Baud115200,
		Baud128000,
		Baud230400,
		Baud256000,
		Baud460800,
		Baud921600
	};

	// Either a standard baud enum or a custom integer
	using BaudSetting = std::variant<BaudRate, unsigned int>;

	struct SerialConfig {
		std::string device;
		BaudSetting baudRate = BaudRate::Baud115200;
		Parity parity = Parity::None;
		StopBits stopBits = StopBits::One;
		FlowControl flowControl = FlowControl::None;
		std::chrono::milliseconds timeout = std::chrono::milliseconds{1000};
		bool non_blocking = false;
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

	   private:
		bool non_blocking = false;

#ifdef _WIN32
		void* handle = nullptr;  // HANDLE
#else
		int fd = -1;
#endif
	};

	// --- Utility: convert baud settings to human-readable strings ---
	std::string to_string(BaudRate rate);
	std::string to_string(const BaudSetting& setting);
}  // namespace DroidWire

#endif  // DROIDWIRE_H