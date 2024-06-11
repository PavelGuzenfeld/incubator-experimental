#include <cstdint>
#include "mavsdk_connector.hpp"
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

class Timesync {
public:
    Timesync(mavsdk::MavlinkPassthrough& mavlink_passthrough);

private:
    void send_timesync_request();
    int64_t calculate_offset() const;
    int64_t local_time() const;
    void on_timesync_message(const mavlink_message_t& message);

    mavsdk::MavlinkPassthrough& _mavlink_passthrough;
    int64_t _send_timestamp;
    int64_t _receive_timestamp;
    int64_t _offset;
};

#include <chrono>
#include <iostream>

Timesync::Timesync(mavsdk::MavlinkPassthrough& mavlink_passthrough)
    : _mavlink_passthrough(mavlink_passthrough), _send_timestamp(0), _receive_timestamp(0), _offset(0) {
    _mavlink_passthrough.subscribe_message(
        MAVLINK_MSG_ID_TIMESYNC, [this](const mavlink_message_t& message) {
            on_timesync_message(message);
        });
}

void Timesync::send_timesync_request() {
    _send_timestamp = local_time();
    mavlink_timesync_t timesync ={0,0};    
    mavlink_message_t message;
    mavlink_msg_timesync_pack(
        _mavlink_passthrough.get_our_sysid(),
        _mavlink_passthrough.get_our_compid(),
        &message,
        timesync.tc1,
        timesync.ts1);
    auto result = _mavlink_passthrough.queue_message([&message](MavlinkAddress, uint8_t)
                                                      { return message; });
}

void Timesync::on_timesync_message(const mavlink_message_t& message) {
    mavlink_timesync_t timesync;
    mavlink_msg_timesync_decode(&message, &timesync);

    if (timesync.tc1 == 0) {
        // this is a request, respond back
        mavlink_message_t response;
        mavlink_msg_timesync_pack(
            _mavlink_passthrough.get_our_sysid(),
            _mavlink_passthrough.get_our_compid(),
            &response,
            local_time(),
            timesync.ts1);
            auto result = _mavlink_passthrough.queue_message([&message](MavlinkAddress, uint8_t)
                                                      { return message; });
    } else {
        // this is a response
        _receive_timestamp = local_time();
        _offset = ((timesync.tc1 - timesync.ts1) + (timesync.ts1 - _send_timestamp)) / 2;
    }
}

int64_t Timesync::calculate_offset() const {
    return _offset;
}

int64_t Timesync::local_time() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

#include <iostream>
#include <thread>

int main(int argc, char** argv) {
     if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <connection_url>\n";
        return 1;
    }

    std::string connection_url = argv[1];
    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration(mavsdk::Mavsdk::ComponentType::CompanionComputer)};
    auto system = connect_to_mavsdk(mavsdk,connection_url);
    if (!system) {
        return 1;
    }

    auto mavlink_passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(system);

    Timesync timesync{mavlink_passthrough};

    std::this_thread::sleep_for(std::chrono::seconds(1));

    timesync.send_timesync_request();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    int64_t offset = timesync.calculate_offset();
    std::cout << "Estimated offset: " << offset << " nanoseconds\n";

    return 0;
}