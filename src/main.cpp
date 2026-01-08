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
static std::unique_ptr<uros::agent::graph_manager::GraphManagerPlugin> g_graph_plugin;

static bool callback_init() { return g_transport && g_transport->init(); }
static bool callback_fini() { return !g_transport || g_transport->fini(); }

static ssize_t callback_send(const eprosima::uxr::CustomEndPoint*, uint8_t* buf, size_t len, eprosima::uxr::TransportRc& err) {
    if (!g_transport || !g_transport->is_connected()) { err = eprosima::uxr::TransportRc::connection_error; return -1; }
    ssize_t r = g_transport->send(buf, len);
    err = (r < 0) ? eprosima::uxr::TransportRc::server_error : eprosima::uxr::TransportRc::ok;
    return r;
}

static ssize_t callback_receive(eprosima::uxr::CustomEndPoint*, uint8_t* buf, size_t max, int timeout_ms, eprosima::uxr::TransportRc& err) {
    if (!g_transport) { err = eprosima::uxr::TransportRc::connection_error; return -1; }
    ssize_t r = g_transport->receive(buf, max, timeout_ms);
    err = (r < 0) ? eprosima::uxr::TransportRc::connection_error : (r == 0) ? eprosima::uxr::TransportRc::timeout_error : eprosima::uxr::TransportRc::ok;
    return r;
}

static void signal_handler(int) { g_running = false; }

struct Args { std::string device; int verbosity = 4; int scan_timeout = 10000; };

static Args parse_args(int argc, char* argv[]) {
    Args a;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if ((arg == "--dev" || arg == "-d") && i + 1 < argc) a.device = argv[++i];
        else if (arg == "-v" && i + 1 < argc) a.verbosity = std::clamp(std::stoi(argv[++i]), 0, 6);
        else if (arg == "--timeout" && i + 1 < argc) a.scan_timeout = std::stoi(argv[++i]);
        else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " --dev <name> [-v <0-6>] [--timeout <ms>]\n";
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

    eprosima::uxr::CustomAgent::InitFunction init_fn = callback_init;
    eprosima::uxr::CustomAgent::FiniFunction fini_fn = callback_fini;
    eprosima::uxr::CustomAgent::SendMsgFunction send_fn = callback_send;
    eprosima::uxr::CustomAgent::RecvMsgFunction recv_fn = callback_receive;

    eprosima::uxr::CustomAgent agent("ble_agent", &g_endpoint, eprosima::uxr::Middleware::Kind::FASTDDS, true,
        init_fn, fini_fn, send_fn, recv_fn);

    agent.set_verbose_level(args.verbosity);

    g_graph_plugin = std::make_unique<uros::agent::graph_manager::GraphManagerPlugin>();
    g_graph_plugin->register_callbacks(agent);

    if (!agent.start()) { std::cerr << "[Agent] Failed to start\n"; return 1; }

    std::cout << "[Agent] Running. Press Ctrl+C to stop." << std::endl;

    while (g_running && g_transport && g_transport->is_connected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[Agent] Stopping..." << std::endl;
    agent.stop();

    g_graph_plugin.reset();
    g_transport.reset();

    std::cout << "[Agent] Done." << std::endl;
    return 0;
}
