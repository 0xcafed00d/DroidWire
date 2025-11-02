#include <droidwire/droidwire.h>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " <serial_device>" << std::endl;
		return 1;
	}
	std::string device = argv[1];

	DroidWire::SerialConfig config;
	config.device = device;
	config.baudRate = DroidWire::BaudRate::B115200;
	config.timeout = std::chrono::milliseconds(100);

	try {
		DroidWire::SerialPort serial(config);

		std::string dataToSend = "Hello, DroidWire!";
		serial.write(std::span<const std::byte>(
		    reinterpret_cast<const std::byte*>(dataToSend.data()), dataToSend.size()));

		std::byte buffer[256];
		std::span<std::byte> bufferSpan(buffer, sizeof(buffer));
		serial.read(bufferSpan);

		std::string receivedData(reinterpret_cast<char*>(buffer), bufferSpan.size());
		std::cout << "Received: " << receivedData << std::endl;
	} catch (const std::exception& ex) {
		std::cerr << "Error: " << ex.what() << std::endl;
		return 1;
	}
	return 0;
}