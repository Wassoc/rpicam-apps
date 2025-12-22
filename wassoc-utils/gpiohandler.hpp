#include <iostream>
#include <string>
#include <vector>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

class GpioHandler {
private:
    int tx_serial_fd;
    int rx_serial_fd;
    bool tx_serial_open;
    bool rx_serial_open;
    bool fire_and_forget;
    unsigned int red_brightness;
    unsigned int green_brightness;
    unsigned int blue_brightness;
    bool illumination_trigger_disabled;
    std::string tx_serial_device = "/dev/ttyAMA5";
    std::string rx_serial_device = "/dev/ttyAMA4";
    std::vector<std::string> lamp_pattern_vec;
    unsigned int lamp_pattern_index;
    
    // Send a command string over serial
    bool sendCommand(const std::string& command) {
        if (!tx_serial_open || tx_serial_fd < 0) {
            return false;
        }
        
        std::string formatted_command = "$," + command + "\r\n";
        ssize_t written = write(tx_serial_fd, formatted_command.c_str(), formatted_command.length());
        if (written < 0) {
            return false;
        }
        
        // Flush to ensure data is sent
        tcdrain(tx_serial_fd);
        return true;
    }

    std::string readResponse() {
        if (!rx_serial_open || rx_serial_fd < 0) {
            return "";
        }
        char buffer[512];
        ssize_t bytes_read = read(rx_serial_fd, buffer, sizeof(buffer));
        if (bytes_read < 0) {
            return "";
        }
        return std::string(buffer, bytes_read);
    }
    
    // Initialize serial port
    bool initSerial(const std::string& device, speed_t baud_rate = B115200, bool is_tx = true) {
        // Open the serial port
        int current_serial_fd;
        if (is_tx) {
            tx_serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY);
            if (tx_serial_fd < 0) {
                return false;
            }
            current_serial_fd = tx_serial_fd;
        } else {
            rx_serial_fd = open(device.c_str(), O_RDWR | O_NOCTTY);
            if (rx_serial_fd < 0) {
                return false;
            }
            current_serial_fd = rx_serial_fd;
        }
        
        // Configure termios structure
        struct termios tty;
        if (tcgetattr(current_serial_fd, &tty) != 0) {
            close(current_serial_fd);
            if (is_tx) {
                tx_serial_fd = -1;
            } else {
                rx_serial_fd = -1;
            }
            return false;
        }
        
        // Set baud rate
        cfsetospeed(&tty, baud_rate);
        cfsetispeed(&tty, baud_rate);
        
        // 8N1: 8 data bits, no parity, 1 stop bit
        tty.c_cflag &= ~PARENB;         // No parity
        tty.c_cflag &= ~CSTOPB;         // 1 stop bit
        tty.c_cflag &= ~CSIZE;          // Clear size bits
        tty.c_cflag |= CS8;              // 8 data bits
        tty.c_cflag &= ~CRTSCTS;         // No hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;   // Enable receiver, ignore modem controls
        
        // Input flags
        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        
        // Output flags
        tty.c_oflag &= ~OPOST;          // Raw output
        
        // Local flags
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); // Raw mode
        
        // Control characters
        tty.c_cc[VMIN] = 0;              // Non-blocking read
        tty.c_cc[VTIME] = 10;         // 1 second timeout

        // Apply settings
        if (tcsetattr(current_serial_fd, TCSANOW, &tty) != 0) {
            close(current_serial_fd);
            if (is_tx) {
                tx_serial_fd = -1;
            } else {
                rx_serial_fd = -1;
            }
            return false;
        }
        
        // Flush any existing data
        tcflush(current_serial_fd, TCIOFLUSH);
        
        if (is_tx) {
            tx_serial_open = true;
        } else {
            rx_serial_open = true;
        }
        return true;
    }

    bool setChannelBrightness(unsigned int channel, unsigned int brightness) {
        if (channel > 3) {
            return false;
        }
        bool success = false;
        int retries = 0;
        std::string command = "l," + std::to_string(channel) + "," + std::to_string(brightness) + ',';
        if (fire_and_forget) {
            sendCommand(command);
            return true;
        }
        while (!success && retries < 3) {
            sendCommand(command);
            std::string response = readResponse();
            if (response.find("OK") != std::string::npos) {
                success = true;
            } else {
                success = false;
            }
            retries++;
        }
        return success;
    }

    bool setActiveChannels(std::string active_channels) {
        bool success = false;
        int retries = 0;
        std::string command = "r," + active_channels;
        if (fire_and_forget) {
            sendCommand(command);
            return true;
        }
        while (!success && retries < 3) {
            sendCommand(command);
            std::string response = readResponse();
            if (response.find("OK") != std::string::npos) {
                success = true;
            } else {
                success = false;
            }
            retries++;
        }
        return success;
    }

    bool turnOffLamp() {
        bool success = false;
        int retries = 0;
        std::string command = "off,";
        if (fire_and_forget) {
            sendCommand(command);
            return true;
        }
        while (!success && retries < 3) {
            sendCommand(command);
            std::string response = readResponse();
            if (response.find("OK") != std::string::npos) {
                success = true;
            }
            retries++;
        }
        return success;
    }

    bool turnOnLamp() {
        bool success = false;
        int retries = 0;
        std::string command = "on,";
        if (fire_and_forget) {
            sendCommand(command);
            return true;
        }
        while (!success && retries < 3) {
            sendCommand(command);
            std::string response = readResponse();
            if (response.find("OK") != std::string::npos) {
                success = true;
            }
            retries++;
        }
        return success;
    }

    bool disableIlluminationTrigger() {
        bool success = false;
        int retries = 0;
        std::string command = "t,0,";
        if (fire_and_forget) {
            sendCommand(command);
            return true;
        }
        while (!success && retries < 3) {
            sendCommand(command);
            std::string response = readResponse();
            if (response.find("OK") != std::string::npos) {
                success = true;
            }
            retries++;
        }
        return success;
    }

    bool enableIlluminationTrigger() {
        bool success = false;
        int retries = 0;
        std::string command = "t,1,";
        if (fire_and_forget) {
            sendCommand(command);
            return true;
        }
        while (!success && retries < 3) {
            sendCommand(command);
            std::string response = readResponse();
            if (response.find("OK") != std::string::npos) {
                success = true;
            }
            retries++;
        }
        return success;
    }

public:
    GpioHandler(std::string lamp_pattern = "R", unsigned int r_brightness = 100, unsigned int g_brightness = 100, unsigned int b_brightness = 100, bool disable_illumination_trigger = false, bool should_fire_and_forget = false, speed_t baud_rate = B9600) {
        tx_serial_fd = -1;
        rx_serial_fd = -1;
        tx_serial_open = false;
        rx_serial_open = false;
        red_brightness = r_brightness;
        green_brightness = g_brightness;
        blue_brightness = b_brightness;
        illumination_trigger_disabled = disable_illumination_trigger;
        fire_and_forget = should_fire_and_forget;
        // Initialize serial port
        if (!initSerial(tx_serial_device, baud_rate, true)) {
            // Failed to open serial port
            return;
        }
        if (!fire_and_forget) {
            if (!initSerial(rx_serial_device, baud_rate, false)) {
                // Failed to open serial port
                fire_and_forget = true;
            }
        }

        setChannelBrightness(0, red_brightness);
        setChannelBrightness(1, green_brightness);
        setChannelBrightness(2, blue_brightness);
        turnOffLamp();
        if (illumination_trigger_disabled) {
            disableIlluminationTrigger();
        } else {
            enableIlluminationTrigger();
        }

        // Parse lamp_pattern into a vector of strings, delimited by ','
        size_t start = 0, end = 0;
        while ((end = lamp_pattern.find(',', start)) != std::string::npos) {
            lamp_pattern_vec.push_back(lamp_pattern.substr(start, end - start));
            start = end + 1;
        }
        lamp_pattern_vec.push_back(lamp_pattern.substr(start));
        lamp_pattern_index = 0;
    }

    ~GpioHandler() {
        closeGpio();
    }

    void setNextLampColor() {
        bool wasColorSet = false;
        if (lamp_pattern_index == lamp_pattern_vec.size()) {
            lamp_pattern_index = 0;
        }
        std::string active_channels = "";
        std::string current_color = lamp_pattern_vec[lamp_pattern_index];
        for (unsigned int i = 0; i < current_color.size(); i++) {
            char letter = current_color[i];
            if (letter == 'R' || letter == 'r') {
                active_channels += "0,";
                wasColorSet = true;
            } else if (letter == 'G' || letter == 'g') {
                active_channels += "1,";
                wasColorSet = true;
            } else if (letter == 'B' || letter == 'b') {
                active_channels += "2,";
                wasColorSet = true;
            } else if (letter == 'W' || letter == 'w') {
                active_channels += "0,1,2,";
                wasColorSet = true;
            }
        }
        if (!wasColorSet) {
            active_channels += "0,";
        }
        setActiveChannels(active_channels);
        if (illumination_trigger_disabled) {
            // sending a 'on' command will update the LED channels to match the set active channels
            turnOnLamp();
        }
        
        lamp_pattern_index++;
    }
    
    void closeGpio() {
        // Turn off all colors before closing
        turnOffLamp();
        
        if (tx_serial_open && tx_serial_fd >= 0) {
            close(tx_serial_fd);
            tx_serial_fd = -1;
            tx_serial_open = false;
        }
        if (rx_serial_open && rx_serial_fd >= 0) {
            close(rx_serial_fd);
            rx_serial_fd = -1;
            rx_serial_open = false;
        }
    }
};