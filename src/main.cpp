#include <memory>
#include <string>
#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include <uxr/agent/transport/custom/CustomAgent.hpp>
#include <uxr/agent/transport/endpoint/CustomEndPoint.hpp>
#include <agent/graph_manager/graph_manager_plugin.hpp>

#include "transport/ble_transport.hpp"

static std::unique_ptr<micro_ros_agent_ble::BLETransport> g_transport;
static eprosima::uxr::CustomEndPoint g_endpoint;
static std::atomic<bool> g_running{true};
static std::atomic<bool> g_shutting_down{false};
static std::unique_ptr<uros::agent::graph_manager::GraphManagerPlugin> g_graph_plugin;

static bool callback_init() { return g_transport && g_transport->init(); }
static bool callback_fini() { return !g_transport || g_transport->fini(); }

static ssize_t callback_send(const eprosima::uxr::CustomEndPoint*, uint8_t* buf, size_t len, eprosima::uxr::TransportRc& err) {
    if (g_shutting_down || !g_transport || !g_transport->is_connected()) { err = eprosima::uxr::TransportRc::connection_error; return -1; }
    ssize_t r = g_transport->send(buf, len);
    err = (r < 0) ? eprosima::uxr::TransportRc::server_error : eprosima::uxr::TransportRc::ok;
    return r;
}

static ssize_t callback_receive(eprosima::uxr::CustomEndPoint*, uint8_t* buf, size_t max, int timeout_ms, eprosima::uxr::TransportRc& err) {
    if (g_shutting_down || !g_transport) { err = eprosima::uxr::TransportRc::connection_error; return -1; }
    ssize_t r = g_transport->receive(buf, max, timeout_ms);
    err = (r < 0) ? eprosima::uxr::TransportRc::connection_error : (r == 0) ? eprosima::uxr::TransportRc::timeout_error : eprosima::uxr::TransportRc::ok;
    return r;
}

static void signal_handler(int) { g_running = false; }

struct Args { std::string device; int verbosity = 4; int scan_timeout = 10000; int reconnect_delay = 3; int rssi_interval = 0; };

static Args parse_args(int argc, char* argv[]) {
    Args a;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "--dev" || arg == "-d") && i + 1 < argc) a.device = argv[++i];
        else if ((arg == "-v" || arg == "--verbose") && i + 1 < argc) a.verbosity = std::clamp(std::stoi(argv[++i]), 0, 6);
        else if (arg == "--timeout" && i + 1 < argc) a.scan_timeout = std::stoi(argv[++i]);
        else if (arg == "--reconnect-delay" && i + 1 < argc) a.reconnect_delay = std::max(0, std::stoi(argv[++i]));
        else if (arg == "--rssi-interval" && i + 1 < argc) a.rssi_interval = std::clamp(std::stoi(argv[++i]), 0, 60);
        else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " --dev <name> [-v|--verbose <0-6>] [--timeout <ms>] [--reconnect-delay <s>] [--rssi-interval <s>]\n";
            std::cout << "  -d, --dev            BLE device name to connect to (required)\n";
            std::cout << "  -v, --verbose        Verbosity level 0-6 (default: 4, >=5 enables transport debug)\n";
            std::cout << "  --timeout            BLE scan timeout in ms (default: 10000)\n";
            std::cout << "  --reconnect-delay    Seconds between reconnection attempts (default: 3, 0 to disable)\n";
            std::cout << "  --rssi-interval      RSSI logging interval in seconds (default: 0 = disabled, max: 60)\n";
            std::exit(0);
        }
    }
    return a;
}

int main(int argc, char* argv[]) {
    Args args = parse_args(argc, argv);
    if (args.device.empty()) { std::cerr << "Error: --dev required\n"; return 1; }

    std::cout << "micro-ROS BLE Agent | Device: " << args.device << std::endl;

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    g_transport = std::make_unique<micro_ros_agent_ble::BLETransport>(args.device, args.scan_timeout);

    // Enable RSSI monitoring
    if (args.rssi_interval > 0) {
        g_transport->set_rssi_interval(args.rssi_interval);
        std::cout << "[Agent] RSSI logging every " << args.rssi_interval << "s" << std::endl;
    }

    // Enable transport debug logging for high verbosity
    if (args.verbosity >= 5) {
        g_transport->set_debug(true);
        std::cout << "[Agent] Transport debug logging enabled (verbosity=" << args.verbosity << ")" << std::endl;
    }

    eprosima::uxr::CustomAgent::InitFunction init_fn = callback_init;
    eprosima::uxr::CustomAgent::FiniFunction fini_fn = callback_fini;
    eprosima::uxr::CustomAgent::SendMsgFunction send_fn = callback_send;
    eprosima::uxr::CustomAgent::RecvMsgFunction recv_fn = callback_receive;

    eprosima::uxr::CustomAgent agent("ble_agent", &g_endpoint, eprosima::uxr::Middleware::Kind::FASTDDS, true,
        init_fn, fini_fn, send_fn, recv_fn);

    agent.set_verbose_level(args.verbosity);
    std::cout << "[Agent] XRCE-DDS verbosity level: " << args.verbosity << std::endl;

    g_graph_plugin = std::make_unique<uros::agent::graph_manager::GraphManagerPlugin>();
    g_graph_plugin->register_callbacks(agent);

    bool reconnect = args.reconnect_delay > 0;

    while (g_running) {
        g_shutting_down = false;
        if (!agent.start()) {
            if (!reconnect || !g_running) {
                std::cerr << "[Agent] Failed to start" << std::endl;
                break;
            }
            std::cout << "[Agent] Failed to start, retrying in "
                      << args.reconnect_delay << "s..." << std::endl;
            for (int i = 0; i < args.reconnect_delay * 10 && g_running; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            continue;
        }

        std::cout << "[Agent] Running. Press Ctrl+C to stop." << std::endl;

        while (g_running && g_transport && g_transport->is_connected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        g_shutting_down = true;
        agent.stop();

        if (!g_running) break;
        if (!reconnect) break;

        std::cout << "[Agent] Connection lost. Reconnecting in "
                  << args.reconnect_delay << "s..." << std::endl;
        for (int i = 0; i < args.reconnect_delay * 10 && g_running; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Final cleanup — set shutdown flag to prevent DDS callbacks from
    // accessing transport after it's torn down (prevents SIGSEGV)
    g_shutting_down = true;
    agent.stop();
    g_graph_plugin.reset();
    g_transport.reset();

    std::cout << "[Agent] Done." << std::endl;
    return 0;
}
