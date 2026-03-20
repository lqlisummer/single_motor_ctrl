#include "app/cli_app.h"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "common/config_loader.h"
#include "domain/single_motor_controller.h"

namespace smc {
namespace {

bool parseMode(const std::string& text, MotorControlMode& mode);
bool parseTheta(const std::string& text, ThetaSource& source);
void printTelemetry(const MotorTelemetry& t);
int printCommandResult(bool ok);

void printUsage() {
    std::cout
        << "motor_cli --config <path> <command> [args]\n\n"
        << "Commands:\n"
        << "  info\n"
        << "  get-id\n"
        << "  enable\n"
        << "  disable\n"
        << "  clear-fault\n"
        << "  zero\n"
        << "  calibrate\n"
        << "  mode <position|velocity|current|pd|brake|openloop>\n"
        << "  set-pos <rad>\n"
        << "  set-vel <rad_s>\n"
        << "  set-cur <amp>\n"
        << "  set-pd-target <pos_rad> <vel_rad_s> <amp>\n"
        << "  strong-drag <force|sensorless|sensor|fusion> <voltage>\n"
        << "  monitor [count] [period_ms] [fast]\n"
        << "  shell\n";
}

int runShell(SingleMotorController& controller, const MotorConfig& motor_cfg) {
    std::cout << "interactive shell started, type help for commands, quit to exit\n";
    std::string line;
    while (std::cout << "> " && std::getline(std::cin, line)) {
        if (line.empty()) {
            continue;
        }
        std::vector<std::string> parts;
        std::string token;
        for (char c : line) {
            if (std::isspace(static_cast<unsigned char>(c))) {
                if (!token.empty()) {
                    parts.push_back(token);
                    token.clear();
                }
            } else {
                token.push_back(c);
            }
        }
        if (!token.empty()) {
            parts.push_back(token);
        }
        if (parts.empty()) {
            continue;
        }
        const std::string& cmd = parts[0];
        if (cmd == "quit" || cmd == "exit") {
            controller.stop();
            return 0;
        }
        if (cmd == "help") {
            printUsage();
            continue;
        }
        if (cmd == "info") {
            auto id = controller.identify();
            std::cout << "model=" << id.model << "\n"
                      << "motor_fw=" << id.firmware_version << "\n"
                      << "board_fw=" << id.board_firmware_version << "\n";
            continue;
        }
        if (cmd == "get-id") {
            std::uint16_t can_id = 0;
            std::uint16_t can_line_id = 0;
            if (!controller.detectMotorAddress(can_id, can_line_id)) { std::cout << "failed\n"; continue; }
            std::cout << "can_id=" << can_id << "\n"
                      << "can_line_id=" << can_line_id << "\n";
            continue;
        }
        if (cmd == "enable") { std::cout << (controller.enable() ? "ok\n" : "failed\n"); continue; }
        if (cmd == "disable") { std::cout << (controller.disable() ? "ok\n" : "failed\n"); continue; }
        if (cmd == "clear-fault") { std::cout << (controller.clearFault() ? "ok\n" : "failed\n"); continue; }
        if (cmd == "zero") { std::cout << (controller.zero() ? "ok\n" : "failed\n"); continue; }
        if (cmd == "calibrate") { std::cout << (controller.calibrate() ? "ok\n" : "failed\n"); continue; }
        if (cmd == "mode" && parts.size() >= 2) {
            MotorControlMode mode{};
            if (!parseMode(parts[1], mode)) { std::cout << "invalid mode\n"; continue; }
            std::cout << (controller.setMode(mode) ? "ok\n" : "failed\n");
            continue;
        }
        if (cmd == "set-pos" && parts.size() >= 2) {
            controller.setMode(MotorControlMode::Position);
            controller.enable();
            controller.moveTo(std::stod(parts[1]));
            std::cout << "ok\n";
            continue;
        }
        if (cmd == "set-vel" && parts.size() >= 2) {
            controller.setMode(MotorControlMode::Velocity);
            controller.enable();
            controller.spin(std::stod(parts[1]));
            std::cout << "ok\n";
            continue;
        }
        if (cmd == "set-cur" && parts.size() >= 2) {
            controller.setMode(MotorControlMode::Current);
            controller.enable();
            controller.applyCurrent(std::stod(parts[1]));
            std::cout << "ok\n";
            continue;
        }
        if (cmd == "set-pd-target" && parts.size() >= 4) {
            controller.setMode(MotorControlMode::Pd);
            controller.enable();
            controller.driver().setPd(motor_cfg.pd_kp, motor_cfg.pd_kd);
            controller.setPdTarget(std::stod(parts[1]), std::stod(parts[2]), std::stod(parts[3]));
            std::cout << "ok\n";
            continue;
        }
        if (cmd == "monitor") {
            const int count = parts.size() >= 2 ? std::stoi(parts[1]) : 10;
            const int period_ms = parts.size() >= 3 ? std::stoi(parts[2]) : 100;
            const bool fast_feedback = parts.size() >= 4 ? (parts[3] == "fast") : false;
            controller.start(MotorControlMode::Position, fast_feedback, period_ms);
            for (int i = 0; i < count; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
                printTelemetry(controller.telemetry());
            }
            controller.stop();
            continue;
        }
        if (cmd == "strong-drag" && parts.size() >= 3) {
            ThetaSource source{};
            if (!parseTheta(parts[1], source)) { std::cout << "invalid strong-drag source\n"; continue; }
            controller.setMode(MotorControlMode::Velocity);
            controller.enable();
            std::cout << (controller.setStrongDragging(source, std::stod(parts[2])) ? "ok\n" : "failed\n");
            continue;
        }
        std::cout << "unknown or incomplete command\n";
    }
    controller.stop();
    return 0;
}

bool parseMode(const std::string& text, MotorControlMode& mode) {
    if (text == "position") mode = MotorControlMode::Position;
    else if (text == "velocity") mode = MotorControlMode::Velocity;
    else if (text == "current") mode = MotorControlMode::Current;
    else if (text == "pd") mode = MotorControlMode::Pd;
    else if (text == "brake") mode = MotorControlMode::Brake;
    else if (text == "openloop") mode = MotorControlMode::OpenLoop;
    else return false;
    return true;
}

bool parseTheta(const std::string& text, ThetaSource& source) {
    if (text == "force") source = ThetaSource::ForceTheta;
    else if (text == "sensorless") source = ThetaSource::SensorLess;
    else if (text == "sensor") source = ThetaSource::Sensor;
    else if (text == "fusion") source = ThetaSource::SensorFusion;
    else return false;
    return true;
}

int printCommandResult(bool ok) {
    if (ok) {
        std::cout << "ok\n";
        return 0;
    }
    std::cout << "failed\n";
    return 4;
}

void printTelemetry(const MotorTelemetry& t) {
    std::cout << std::fixed << std::setprecision(6)
              << "pos=" << t.position_rad
              << " vel=" << t.velocity_rad_s
              << " cur=" << t.current_a
              << " tor_l=" << t.torque_load_nm
              << " tor_e=" << t.torque_elec_nm
              << " encoder=" << t.encoder_value
              << "\n";
}

bool bootstrapController(SingleMotorController& controller) {
    return controller.initialize();
}

}  // namespace

int CliApp::run(int argc, char** argv) {
    if (argc < 2) {
        printUsage();
        return 1;
    }

    std::string config_path = "configs/motor_cli.conf";
    std::vector<std::string> args(argv + 1, argv + argc);

    for (auto it = args.begin(); it != args.end();) {
        if (*it == "--config") {
            if (std::next(it) == args.end()) {
                std::cerr << "missing value for --config\n";
                return 1;
            }
            config_path = *std::next(it);
            it = args.erase(it, std::next(it, 2));
        } else if (*it == "--help" || *it == "-h") {
            printUsage();
            return 0;
        } else {
            ++it;
        }
    }

    if (args.empty()) {
        printUsage();
        return 1;
    }

    BoardConfig board_cfg;
    MotorConfig motor_cfg;
    std::string error;
    if (!ConfigLoader::load(config_path, board_cfg, motor_cfg, &error)) {
        std::cerr << error << "\n";
        return 2;
    }

    SingleMotorController controller(board_cfg, motor_cfg);
    if (!bootstrapController(controller)) {
        std::cerr << "failed to initialize controller\n";
        return 3;
    }

    const std::string cmd = args[0];

    if (cmd == "info") {
        auto id = controller.identify();
        std::cout << "model=" << id.model << "\n"
                  << "motor_fw=" << id.firmware_version << "\n"
                  << "board_fw=" << id.board_firmware_version << "\n";
        return 0;
    }

    if (cmd == "get-id") {
        std::uint16_t can_id = 0;
        std::uint16_t can_line_id = 0;
        if (!controller.detectMotorAddress(can_id, can_line_id)) {
            std::cerr << "failed to detect motor id\n";
            return 4;
        }
        std::cout << "can_id=" << can_id << "\n"
                  << "can_line_id=" << can_line_id << "\n";
        return 0;
    }

    if (cmd == "enable") {
        return printCommandResult(controller.enable());
    }
    if (cmd == "disable") {
        return printCommandResult(controller.disable());
    }
    if (cmd == "clear-fault") {
        return printCommandResult(controller.clearFault());
    }
    if (cmd == "zero") {
        return printCommandResult(controller.zero());
    }
    if (cmd == "calibrate") {
        return printCommandResult(controller.calibrate());
    }
    if (cmd == "mode") {
        if (args.size() < 2) {
            std::cerr << "mode command requires one argument\n";
            return 1;
        }
        MotorControlMode mode{};
        if (!parseMode(args[1], mode)) {
            std::cerr << "invalid mode\n";
            return 1;
        }
        return printCommandResult(controller.setMode(mode));
    }
    if (cmd == "set-pos") {
        if (args.size() < 2) {
            std::cerr << "set-pos requires <rad>\n";
            return 1;
        }
        if (!controller.setMode(MotorControlMode::Position)) {
            return printCommandResult(false);
        }
        if (!controller.enable()) {
            return printCommandResult(false);
        }
        controller.moveTo(std::stod(args[1]));
        return printCommandResult(true);
    }
    if (cmd == "set-vel") {
        if (args.size() < 2) {
            std::cerr << "set-vel requires <rad_s>\n";
            return 1;
        }
        if (!controller.setMode(MotorControlMode::Velocity)) {
            return printCommandResult(false);
        }
        if (!controller.enable()) {
            return printCommandResult(false);
        }
        controller.spin(std::stod(args[1]));
        return printCommandResult(true);
    }
    if (cmd == "set-cur") {
        if (args.size() < 2) {
            std::cerr << "set-cur requires <amp>\n";
            return 1;
        }
        if (!controller.setMode(MotorControlMode::Current)) {
            return printCommandResult(false);
        }
        if (!controller.enable()) {
            return printCommandResult(false);
        }
        controller.applyCurrent(std::stod(args[1]));
        return printCommandResult(true);
    }
    if (cmd == "set-pd-target") {
        if (args.size() < 4) {
            std::cerr << "set-pd-target requires <pos> <vel> <cur>\n";
            return 1;
        }
        if (!controller.setMode(MotorControlMode::Pd)) {
            return printCommandResult(false);
        }
        if (!controller.enable()) {
            return printCommandResult(false);
        }
        controller.driver().setPd(motor_cfg.pd_kp, motor_cfg.pd_kd);
        controller.setPdTarget(std::stod(args[1]), std::stod(args[2]), std::stod(args[3]));
        return printCommandResult(true);
    }
    if (cmd == "strong-drag") {
        if (args.size() < 3) {
            std::cerr << "strong-drag requires <force|sensorless|sensor|fusion> <voltage>\n";
            return 1;
        }
        ThetaSource source{};
        if (!parseTheta(args[1], source)) {
            std::cerr << "invalid strong-drag source\n";
            return 1;
        }
        if (!controller.setMode(MotorControlMode::Velocity)) {
            return printCommandResult(false);
        }
        if (!controller.enable()) {
            return printCommandResult(false);
        }
        return printCommandResult(controller.setStrongDragging(source, std::stod(args[2])));
    }
    if (cmd == "shell") {
        return runShell(controller, motor_cfg);
    }
    if (cmd == "monitor") {
        const int count = args.size() >= 2 ? std::stoi(args[1]) : 10;
        const int period_ms = args.size() >= 3 ? std::stoi(args[2]) : 100;
        const bool fast_feedback = args.size() >= 4 ? (args[3] == "fast") : false;
        controller.start(MotorControlMode::Position, fast_feedback, period_ms);
        for (int i = 0; i < count; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
            printTelemetry(controller.telemetry());
        }
        controller.stop();
        return 0;
    }

    std::cerr << "unknown command: " << cmd << "\n";
    printUsage();
    return 1;
}

}  // namespace smc
