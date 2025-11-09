
#include <chrono>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <thread>

#include "droidwire/droidwire.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#endif

#if defined(__linux__) && defined(BOTHER)
// For custom baud rates on Linux using termios2
#include <asm/termbits.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#endif

namespace DroidWire {

	// ------------------ Helper: map baud rate ------------------

#ifdef _WIN32
	static DWORD toWindowsBaud(const BaudSetting& setting) {
		if (std::holds_alternative<unsigned int>(setting))
			return std::get<unsigned int>(setting);

		switch (std::get<BaudRate>(setting)) {
			case BaudRate::Baud110:
				return CBR_110;
			case BaudRate::Baud300:
				return CBR_300;
			case BaudRate::Baud600:
				return CBR_600;
			case BaudRate::Baud1200:
				return CBR_1200;
			case BaudRate::Baud2400:
				return CBR_2400;
			case BaudRate::Baud4800:
				return CBR_4800;
			case BaudRate::Baud9600:
				return CBR_9600;
			case BaudRate::Baud14400:
				return CBR_14400;
			case BaudRate::Baud19200:
				return CBR_19200;
			case BaudRate::Baud38400:
				return CBR_38400;
			case BaudRate::Baud57600:
				return CBR_57600;
			case BaudRate::Baud115200:
				return CBR_115200;
			case BaudRate::Baud128000:
				return CBR_128000;
			case BaudRate::Baud230400:
				return 230400;
			case BaudRate::Baud256000:
				return 256000;
			case BaudRate::Baud460800:
				return 460800;
			case BaudRate::Baud921600:
				return 921600;
			default:
				throw std::invalid_argument("Unsupported baud rate");
		}
	}
#else
	static speed_t toPosixBaud(const BaudSetting& setting) {
		if (std::holds_alternative<unsigned int>(setting)) {
#ifdef BOTHER
			return BOTHER;  // we’ll use termios2 for custom baud
#else
			throw std::invalid_argument("Custom baud rates unsupported on this platform");
#endif
		}

		switch (std::get<BaudRate>(setting)) {
			case BaudRate::Baud110:
				return B110;
			case BaudRate::Baud300:
				return B300;
			case BaudRate::Baud600:
				return B600;
			case BaudRate::Baud1200:
				return B1200;
			case BaudRate::Baud2400:
				return B2400;
			case BaudRate::Baud4800:
				return B4800;
			case BaudRate::Baud9600:
				return B9600;
			case BaudRate::Baud19200:
				return B19200;
			case BaudRate::Baud38400:
				return B38400;
			case BaudRate::Baud57600:
				return B57600;
			case BaudRate::Baud115200:
				return B115200;
#ifdef B230400
			case BaudRate::Baud230400:
				return B230400;
#endif
#ifdef B460800
			case BaudRate::Baud460800:
				return B460800;
#endif
#ifdef B921600
			case BaudRate::Baud921600:
				return B921600;
#endif
			default:
				throw std::invalid_argument("Unsupported baud rate");
		}
	}
#endif

	// ------------------ SerialPort constructor ------------------
#ifdef _WIN32
	SerialPort::SerialPort(const SerialConfig& cfg) {
		non_blocking = cfg.non_blocking;

		std::wstring wdevice(cfg.device.begin(), cfg.device.end());
		DWORD flags = cfg.non_blocking ? FILE_FLAG_OVERLAPPED : 0;

		handle = CreateFileW(wdevice.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
		                     OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL | flags, nullptr);
		if (handle == INVALID_HANDLE_VALUE)
			throw std::system_error(GetLastError(), std::system_category(),
			                        "Failed to open serial port");

		DCB dcb{};
		dcb.DCBlength = sizeof(dcb);
		if (!GetCommState(handle, &dcb))
			throw std::system_error(GetLastError(), std::system_category(), "GetCommState failed");

		dcb.BaudRate = toWindowsBaud(cfg.baudRate);
		dcb.ByteSize = 8;
		dcb.StopBits = (cfg.stopBits == StopBits::Two) ? TWOSTOPBITS : ONESTOPBIT;
		dcb.Parity = (cfg.parity == Parity::Even)  ? EVENPARITY
		             : (cfg.parity == Parity::Odd) ? ODDPARITY
		                                           : NOPARITY;

		dcb.fOutxCtsFlow = (cfg.flowControl == FlowControl::Hardware);
		dcb.fRtsControl = (cfg.flowControl == FlowControl::Hardware) ? RTS_CONTROL_HANDSHAKE
		                                                             : RTS_CONTROL_DISABLE;
		dcb.fOutX = (cfg.flowControl == FlowControl::Software);
		dcb.fInX = (cfg.flowControl == FlowControl::Software);

		if (!SetCommState(handle, &dcb))
			throw std::system_error(GetLastError(), std::system_category(), "SetCommState failed");

		COMMTIMEOUTS timeouts{};
		timeouts.ReadIntervalTimeout = (DWORD)cfg.timeout.count();
		timeouts.ReadTotalTimeoutConstant = (DWORD)cfg.timeout.count();
		SetCommTimeouts(handle, &timeouts);
	}

#else
	SerialPort::SerialPort(const SerialConfig& cfg) {
		non_blocking = cfg.non_blocking;
		fd = open(cfg.device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
		if (fd < 0)
			throw std::system_error(errno, std::generic_category(), "Failed to open serial port");

		if (cfg.non_blocking)
			fcntl(fd, F_SETFL, O_NONBLOCK);

#ifdef BOTHER
		// Try modern Linux path (termios2 with BOTHER)
		termios2 tty{};
		if (ioctl(fd_, TCGETS2, &tty) == 0) {
			tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
			tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
			tty.c_cflag &= ~CBAUD;
			tty.c_cflag |= BOTHER;

			if (std::holds_alternative<unsigned int>(cfg.baudRate)) {
				unsigned int custom = std::get<unsigned int>(cfg.baudRate);
				tty.c_ispeed = custom;
				tty.c_ospeed = custom;
			} else {
				speed_t baud = toPosixBaud(cfg.baudRate);
				tty.c_ispeed = baud;
				tty.c_ospeed = baud;
			}

			if (ioctl(fd_, TCSETS2, &tty) != 0)
				throw std::system_error(errno, std::generic_category(),
				                        "Failed to apply termios2 settings");
		} else
#endif
		{
			// Fallback: legacy termios (no BOTHER)
			termios tty{};
			if (tcgetattr(fd, &tty) != 0)
				throw std::system_error(errno, std::generic_category(), "tcgetattr failed");

			speed_t baud = toPosixBaud(cfg.baudRate);
			cfsetospeed(&tty, baud);
			cfsetispeed(&tty, baud);

			tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
			tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;

			if (tcsetattr(fd, TCSANOW, &tty) != 0)
				throw std::system_error(errno, std::generic_category(), "tcsetattr failed");
		}
	}
#endif

#ifdef _WIN32
	SerialPort::~SerialPort() {
		if (handle && handle != INVALID_HANDLE_VALUE)
			CloseHandle(handle);
	}

#else
	SerialPort::~SerialPort() {
		if (fd >= 0)
			close(fd);
	}
#endif

#ifdef _WIN32
	SerialPort::SerialPort(SerialPort&& o) noexcept {
		handle = o.handle;
		o.handle = nullptr;
		non_blocking = o.non_blocking;
	}
#else
	SerialPort::SerialPort(SerialPort&& o) noexcept {
		fd = o.fd;
		o.fd = -1;
		non_blocking = o.non_blocking;
	}
#endif

#ifdef _WIN32
	SerialPort& SerialPort::operator=(SerialPort&& o) noexcept {
		if (this != &o) {
			this->~SerialPort();
			handle = o.handle;
			o.handle = nullptr;
			non_blocking = o.non_blocking;
		}
		return *this;
	}
#else
	SerialPort& SerialPort::operator=(SerialPort&& o) noexcept {
		if (this != &o) {
			this->~SerialPort();
			fd = o.fd;
			o.fd = -1;
			non_blocking = o.non_blocking;
		}
		return *this;
	}
#endif

	std::span<const std::byte> SerialPort::write(std::span<const std::byte> data) {
#ifdef _WIN32
		DWORD written{0};

		if (non_blocking) {
			OVERLAPPED ovl{};
			ZeroMemory(&ovl, sizeof(ovl));

			if (!WriteFile(handle, data.data(), (DWORD)data.size(), &written, &ovl)) {
				if (GetLastError() != ERROR_IO_PENDING)
					throw std::system_error(GetLastError(), std::system_category(),
					                        "WriteFile failed");
			}

		} else {
			if (!WriteFile(handle, data.data(), (DWORD)data.size(), &written, nullptr))
				throw std::system_error(GetLastError(), std::system_category(), "WriteFile failed");
		}
		return data.subspan(0, written);
#else
		ssize_t ret = ::write(fd, data.data(), data.size());
		if (ret < 0 && errno != EAGAIN)
			throw std::system_error(errno, std::generic_category(), "write failed");
		return data.first(static_cast<std::size_t>(std::max<ssize_t>(0, ret)));
#endif
	}

#ifdef _WIN32
	std::span<std::byte> SerialPort::read(std::span<std::byte> buffer) {
		DWORD bytesRead = 0;
		if (non_blocking) {
			OVERLAPPED ovl{};
			ZeroMemory(&ovl, sizeof(ovl));
			if (!ReadFile(handle, buffer.data(), (DWORD)buffer.size(), &bytesRead, &ovl)) {
				if (GetLastError() != ERROR_IO_PENDING)
					throw std::system_error(GetLastError(), std::system_category(),
					                        "ReadFile failed");
			}
		} else {
			if (!ReadFile(handle, buffer.data(), (DWORD)buffer.size(), &bytesRead, nullptr))
				throw std::system_error(GetLastError(), std::system_category(), "ReadFile failed");
		}
		return buffer.first(bytesRead);
	}

#else

	std::span<std::byte> SerialPort::read(std::span<std::byte> buffer) {
		if (non_blocking) {
			fd_set set;
			FD_ZERO(&set);
			FD_SET(fd, &set);
			timeval tv{};
			tv.tv_sec = 0;
			tv.tv_usec = 0;
			int rv = select(fd + 1, &set, nullptr, nullptr, &tv);
			if (rv <= 0)
				return buffer.first(0);  // return zero bytes read
		}

		ssize_t bytesRead = ::read(fd, buffer.data(), buffer.size());
		if (bytesRead < 0 && errno != EAGAIN)
			throw std::system_error(errno, std::generic_category(), "read failed");
		return buffer.first(static_cast<std::size_t>(std::max<ssize_t>(0, bytesRead)));
	}
#endif

	// Convert a BaudRate enum to its numeric string representation
	std::string to_string(BaudRate rate) {
		switch (rate) {
			case BaudRate::Baud110:
				return "110";
			case BaudRate::Baud300:
				return "300";
			case BaudRate::Baud600:
				return "600";
			case BaudRate::Baud1200:
				return "1200";
			case BaudRate::Baud2400:
				return "2400";
			case BaudRate::Baud4800:
				return "4800";
			case BaudRate::Baud9600:
				return "9600";
			case BaudRate::Baud14400:
				return "14400";
			case BaudRate::Baud19200:
				return "19200";
			case BaudRate::Baud38400:
				return "38400";
			case BaudRate::Baud57600:
				return "57600";
			case BaudRate::Baud115200:
				return "115200";
			case BaudRate::Baud128000:
				return "128000";
			case BaudRate::Baud230400:
				return "230400";
			case BaudRate::Baud256000:
				return "256000";
			case BaudRate::Baud460800:
				return "460800";
			case BaudRate::Baud921600:
				return "921600";
			default:
				return "Unknown";
		}
	}

	// Convert a BaudSetting (either enum or custom) to a string
	std::string to_string(const BaudSetting& setting) {
		if (std::holds_alternative<BaudRate>(setting))
			return to_string(std::get<BaudRate>(setting));

		std::ostringstream oss;
		oss << std::get<unsigned int>(setting);
		return oss.str();
	}

}  // namespace DroidWire
