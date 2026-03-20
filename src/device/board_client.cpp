#include "device/board_client.h"

#include <array>
#include <utility>

extern "C" {
#include "RobotControl.h"
}

namespace smc {

BoardClient::BoardClient(BoardConfig config) : config_(std::move(config)) {}

BoardClient::~BoardClient() {
    disconnect();
}

bool BoardClient::connect() {
    if (connected_ && ctx_ != nullptr) {
        return true;
    }

    ctx_ = robot_create(config_.dev_id);
    if (ctx_ == nullptr) {
        connected_ = false;
        return false;
    }

    const int ok = robot_config_net(ctx_,
                                    config_.local_ip.c_str(),
                                    config_.local_port,
                                    config_.remote_port,
                                    config_.remote_ip.c_str());
    if (!ok) {
        robot_destroy(ctx_);
        ctx_ = nullptr;
        connected_ = false;
        return false;
    }

    connected_ = true;
    if (config_.fast_mode_enabled) {
        setFastMode(true, config_.fast_mode_hz);
    }
    return true;
}

void BoardClient::disconnect() {
    if (ctx_ != nullptr) {
        robot_destroy(ctx_);
        ctx_ = nullptr;
    }
    connected_ = false;
}

bool BoardClient::isConnected() const noexcept {
    return connected_ && ctx_ != nullptr;
}

bool BoardClient::setFastMode(bool enable, int hz) {
    if (ctx_ == nullptr) {
        return false;
    }
    return robot_set_fast_mode(ctx_, enable ? 1 : 0, hz) != 0;
}

std::string BoardClient::queryFirmwareVersion() const {
    if (ctx_ == nullptr) {
        return {};
    }
    std::array<char, 128> buffer{};
    const int ok = robot_motor_get_mother_board_firmware_version(ctx_, buffer.data());
    if (!ok) {
        return {};
    }
    return std::string(buffer.data());
}

std::string BoardClient::sdkVersion() const {
    char* version = get_sdk_version();
    if (version == nullptr) {
        return {};
    }
    return std::string(version);
}

RobotCtx* BoardClient::raw() const noexcept {
    return ctx_;
}

const BoardConfig& BoardClient::config() const noexcept {
    return config_;
}

}  // namespace smc
