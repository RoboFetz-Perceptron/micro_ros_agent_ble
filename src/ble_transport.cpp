#include "transport/ble_transport.hpp"

#include <iostream>
#include <algorithm>
#include <thread>
#include <chrono>

namespace micro_ros_agent_ble {

BLETransport::BLETransport(const std::string& device_name, int scan_timeout_ms)
    : device_name_(device_name)
    , scan_timeout_ms_(scan_timeout_ms)
{
}

BLETransport::~BLETransport()
{
    fini();
}

bool BLETransport::init()
{
    try {
        if (!SimpleBLE::Adapter::bluetooth_enabled()) {
            std::cerr << "[BLE] Bluetooth is not enabled" << std::endl;
            return false;
        }

        auto adapters = SimpleBLE::Adapter::get_adapters();
        if (adapters.empty()) {
            std::cerr << "[BLE] No Bluetooth adapters found" << std::endl;
            return false;
        }

        adapter_ = std::make_unique<SimpleBLE::Adapter>(adapters[0]);
        std::cout << "[BLE] Using adapter: " << adapter_->identifier() << std::endl;

        if (!scan_for_device()) {
            std::cerr << "[BLE] Device '" << device_name_ << "' not found" << std::endl;
            return false;
        }

        if (!connect_to_device()) {
            std::cerr << "[BLE] Failed to connect" << std::endl;
            return false;
        }

        if (!setup_notifications()) {
            std::cerr << "[BLE] Failed to setup notifications" << std::endl;
            if (peripheral_ && peripheral_->is_connected()) {
                peripheral_->disconnect();
            }
            return false;
        }

        connected_ = true;
        std::cout << "[BLE] Connected to " << device_name_
                  << " (" << device_address_ << ")" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[BLE] Init error: " << e.what() << std::endl;
        return false;
    }
}

bool BLETransport::fini()
{
    connected_ = false;
    rx_cv_.notify_all();  // Wake up any waiting receive()

    try {
        if (peripheral_ && peripheral_->is_connected()) {
            try {
                peripheral_->unsubscribe(NUSConfig::SERVICE_UUID, NUSConfig::TX_CHAR_UUID);
            } catch (...) {}
            try {
                peripheral_->disconnect();
            } catch (...) {}
            std::cout << "[BLE] Disconnected" << std::endl;
        }
    } catch (...) {}

    peripheral_.reset();
    adapter_.reset();

    // Clear receive buffer
    std::lock_guard<std::mutex> lock(rx_mutex_);
    std::queue<uint8_t> empty;
    std::swap(rx_buffer_, empty);

    return true;
}

ssize_t BLETransport::send(const uint8_t* buffer, size_t length)
{
    if (!buffer || length == 0 || !connected_ || !peripheral_) {
        return -1;
    }

    try {
        size_t bytes_sent = 0;
        while (bytes_sent < length && connected_) {
            size_t chunk_size = std::min(BLE_CHUNK_SIZE, length - bytes_sent);

            // SimpleBLE::ByteArray is std::string
            SimpleBLE::ByteArray data(
                reinterpret_cast<const char*>(buffer + bytes_sent),
                chunk_size);

            peripheral_->write_command(
                NUSConfig::SERVICE_UUID,
                NUSConfig::RX_CHAR_UUID,
                data);

            bytes_sent += chunk_size;

            // Small delay between chunks to prevent buffer overflow
            if (bytes_sent < length) {
                std::this_thread::sleep_for(std::chrono::milliseconds(CHUNK_DELAY_MS));
            }
        }
        return static_cast<ssize_t>(bytes_sent);

    } catch (const std::exception& e) {
        std::cerr << "[BLE] Send error: " << e.what() << std::endl;
        return -1;
    }
}

ssize_t BLETransport::receive(uint8_t* buffer, size_t max_length, int timeout_ms)
{
    if (!buffer || max_length == 0) {
        return -1;
    }

    // Throttle when disconnected to avoid busy-loop
    if (!connected_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return -1;
    }

    std::unique_lock<std::mutex> lock(rx_mutex_);

    if (rx_buffer_.empty()) {
        auto timeout = std::chrono::milliseconds(timeout_ms);
        bool got_data = rx_cv_.wait_for(lock, timeout, [this]() {
            return !rx_buffer_.empty() || !connected_;
        });

        if (!connected_) return -1;
        if (!got_data) return 0;  // Timeout
    }

    // Copy available data
    size_t bytes_read = 0;
    while (!rx_buffer_.empty() && bytes_read < max_length) {
        buffer[bytes_read++] = rx_buffer_.front();
        rx_buffer_.pop();
    }

    return static_cast<ssize_t>(bytes_read);
}

bool BLETransport::scan_for_device()
{
    std::cout << "[BLE] Scanning for: " << device_name_ << std::endl;

    bool found = false;
    SimpleBLE::Peripheral found_peripheral;

    adapter_->set_callback_on_scan_found([&](SimpleBLE::Peripheral peripheral) {
        if (peripheral.identifier() == device_name_) {
            found_peripheral = peripheral;
            found = true;
            adapter_->scan_stop();
        }
    });

    adapter_->scan_start();

    auto start = std::chrono::steady_clock::now();
    while (!found) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= scan_timeout_ms_) {
            adapter_->scan_stop();
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (found) {
        peripheral_ = std::make_unique<SimpleBLE::Peripheral>(found_peripheral);
        device_address_ = peripheral_->address();
        std::cout << "[BLE] Found: " << device_name_ << " at " << device_address_ << std::endl;
    }

    return found;
}

bool BLETransport::connect_to_device()
{
    if (!peripheral_) return false;

    try {
        std::cout << "[BLE] Connecting..." << std::endl;

        peripheral_->set_callback_on_disconnected([this]() {
            this->on_disconnect();
        });

        peripheral_->connect();

        int wait_ms = 0;
        while (!peripheral_->is_connected() && wait_ms < CONNECT_TIMEOUT_MS) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            wait_ms += 100;
        }

        return peripheral_->is_connected();

    } catch (const std::exception& e) {
        std::cerr << "[BLE] Connection error: " << e.what() << std::endl;
        return false;
    }
}

bool BLETransport::setup_notifications()
{
    if (!peripheral_ || !peripheral_->is_connected()) return false;

    try {
        peripheral_->notify(
            NUSConfig::SERVICE_UUID,
            NUSConfig::TX_CHAR_UUID,
            [this](SimpleBLE::ByteArray data) {
                this->on_notification(data);
            });

        std::cout << "[BLE] Subscribed to notifications" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[BLE] Notification setup error: " << e.what() << std::endl;
        return false;
    }
}

void BLETransport::on_notification(SimpleBLE::ByteArray data)
{
    std::lock_guard<std::mutex> lock(rx_mutex_);

    for (size_t i = 0; i < data.size(); ++i) {
        if (rx_buffer_.size() >= MAX_RX_BUFFER_SIZE) {
            rx_buffer_.pop();  // Drop oldest on overflow
        }
        rx_buffer_.push(static_cast<uint8_t>(data[i]));
    }

    rx_cv_.notify_one();
}

void BLETransport::on_disconnect()
{
    std::cout << "[BLE] Disconnected from " << device_name_ << std::endl;
    connected_ = false;
    rx_cv_.notify_all();
}

}  // namespace micro_ros_agent_ble
