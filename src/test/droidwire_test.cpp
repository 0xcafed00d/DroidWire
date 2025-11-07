#include <droidwire/droidwire.h>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
	/*	if (argc < 2) {
	        std::cerr << "Usage: " << argv[0] << " <serial_device>" << std::endl;
	        return 1;
	    } */

#ifdef _WIN32
	std::string device = "COM3";
#else
	std::string device = "/dev/ttyUSB0";
#endif

	DroidWire::SerialConfig config;
	config.device = device;
	config.baudRate = DroidWire::BaudRate::Baud300;
	config.timeout = std::chrono::milliseconds(0);
	config.async = true;

	try {
		DroidWire::SerialPort serial(config);

		std::string dataToSend =
		    "Hello, "
		    "DroidWire!"
		    "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz"
		    "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz";

		// time how long the write takes
		auto start = std::chrono::high_resolution_clock::now();

		serial.write(std::span<const std::byte>(
		    reinterpret_cast<const std::byte*>(dataToSend.data()), dataToSend.size()));
		auto end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		std::cout << "Write took " << duration << " ms" << std::endl;

		std::byte buffer[256];
		std::span<std::byte> bufferSpan(buffer, sizeof(buffer));

		// time how long the read takes
		start = std::chrono::high_resolution_clock::now();
		auto bytesRead = serial.read(bufferSpan);
		end = std::chrono::high_resolution_clock::now();
		duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
		std::cout << "Read took " << duration << " ms" << std::endl;

		std::string receivedData(reinterpret_cast<char*>(buffer), bytesRead.size());
		std::cout << "Received: " << receivedData << std::endl;
	} catch (const std::exception& ex) {
		std::cerr << "Error: " << ex.what() << std::endl;
		return 1;
	}
	return 0;
}