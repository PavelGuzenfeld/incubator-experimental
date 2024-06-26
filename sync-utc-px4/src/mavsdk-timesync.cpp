#include <cstdint>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
#include <memory>
#include <thread>
#include <functional>

class Timesync {
public:
    Timesync(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough);

    void send_timesync_request(uint8_t target_system, uint8_t target_component);
    [[nodiscard]] int64_t calculate_offset() const;

private:
    int64_t local_time_ns() const;
    void on_timesync_message(const mavlink_message_t& message);

    std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough_;
    int64_t send_timestamp_;
    int64_t receive_timestamp_;
    int64_t offset_;
};

Timesync::Timesync(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough)
    : mavlink_passthrough_(std::move(mavlink_passthrough)), send_timestamp_(0), receive_timestamp_(0), offset_(0) {
    mavlink_passthrough_->subscribe_message(
        MAVLINK_MSG_ID_TIMESYNC, [this](const mavlink_message_t& message) {
            on_timesync_message(message);
        });
}

void Timesync::send_timesync_request(uint8_t target_system, uint8_t target_component) {
    send_timestamp_ = local_time_ns();
    mavlink_message_t message;
    mavlink_timesync_t timesync = {};
    timesync.tc1 = 0;  // This is a request, so tc1 is set to 0
    timesync.ts1 = send_timestamp_;

    mavlink_msg_timesync_pack(
        mavlink_passthrough_->get_our_sysid(),
        mavlink_passthrough_->get_our_compid(),
        &message,
        timesync.tc1,
        timesync.ts1,
        target_system,
        target_component);

    auto result = mavlink_passthrough_->send_message(message);
    if (result != mavsdk::MavlinkPassthrough::Result::Success) {
        std::cerr << "Failed to send timesync request\n";
    }
}

void Timesync::on_timesync_message(const mavlink_message_t& message) {
    mavlink_timesync_t timesync;
    mavlink_msg_timesync_decode(&message, &timesync);

    if (timesync.tc1 == 0) {
        // This is a request, respond back
        mavlink_message_t response;
        mavlink_msg_timesync_pack(
            mavlink_passthrough_->get_our_sysid(),
            mavlink_passthrough_->get_our_compid(),
            &response,
            local_time_ns(),
            timesync.ts1,
            message.sysid,
            message.compid);

        auto result = mavlink_passthrough_->send_message(response);
        if (result != mavsdk::MavlinkPassthrough::Result::Success) {
            std::cerr << "Failed to send timesync response\n";
        }
    } else {
        // This is a response
        receive_timestamp_ = local_time_ns();
        offset_ = ((timesync.tc1 - timesync.ts1) + (timesync.ts1 - send_timestamp_)) / 2;
    }
}

[[nodiscard]] int64_t Timesync::calculate_offset() const {
    return offset_;
}

int64_t Timesync::local_time_ns() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <connection_url>\n";
        return 1;
    }

    std::string connection_url = argv[1];
    mavsdk::Mavsdk mavsdk;
    auto system = connect_to_mavsdk(mavsdk, connection_url);
    if (!system) {
        return 1;
    }

    auto mavlink_passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(system);

    Timesync timesync{mavlink_passthrough};

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Send a timesync request to a specific target system and component
    timesync.send_timesync_request(1, 1); // Example target_system and target_component IDs

    std::this_thread::sleep_for(std::chrono::seconds(2));

    int64_t offset = timesync.calculate_offset();
    std::cout << "Estimated offset: " << offset << " nanoseconds\n";

    return 0;
}
