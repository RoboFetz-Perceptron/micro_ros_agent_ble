#ifndef TRANSPORT_BLE_TRANSPORT_HPP_
#define TRANSPORT_BLE_TRANSPORT_HPP_

#include <simpleble/SimpleBLE.h>

#include <string>
#include <cstdint>
#include <atomic>
#include <memory>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace micro_ros_agent_ble {

    struct NUSConfig {
        static constexpr const char* SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
        static constexpr const char* RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
        static constexpr const char* TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";
    };

    class BLETransport {
    public:
        explicit BLETransport(const std::string& device_name, int scan_timout_ms = 10000);
        ~BLETransport();

        BLETransport(const BLETransport&) = delete;
        BLETransport& operator=(const BLETransport&)= delete;

        bool init();
        bool fini();
        ssize_t send(const uint8_t* buffer, size_t length);
        ssize_t receive(uint8_t* buffer, size_t max_length, int timeout_ms);

        bool is_connected() const { return connected_.load(); }

        // Debug logging control
        void set_debug(bool enable) { debug_enabled_ = enable; }

        std::string device_name() const { return device_name_; }
        std::string device_address() const { return device_address_; }

    private:
        bool scan_for_device();
        bool connect_to_device();
        bool setup_notifications();
        void on_notification(SimpleBLE::ByteArray data);
        void on_disconnect();

        std::string device_name_;
        int scan_timeout_ms_;

        std::unique_ptr<SimpleBLE::Adapter> adapter_;
        std::unique_ptr<SimpleBLE::Peripheral> peripheral_;

        std::queue<uint8_t> rx_buffer_;
        std::mutex rx_mutex_;
        std::condition_variable rx_cv_;

        std::atomic<bool> connected_{false};
        std::string device_address_;
        bool debug_enabled_{false};

        static constexpr size_t MAX_RX_BUFFER_SIZE = 8192;
        static constexpr size_t BLE_CHUNK_SIZE = 244;  // Max ATT payload with 247 MTU
        static constexpr int CHUNK_DELAY_MS = 1;       // Minimal delay between chunks
        static constexpr int CONNECT_TIMEOUT_MS = 10000;

        // Debug helper
        void log_bytes(const char* prefix, const uint8_t* data, size_t len) const;
    };

}

#endif // TRANSPORT_BLE_TRANSPORT_HPP_
