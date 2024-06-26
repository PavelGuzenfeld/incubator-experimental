#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <functional>

// Assuming you have a declaration for this function somewhere
std::shared_ptr<mavsdk::System> connect_to_mavsdk(mavsdk::Mavsdk& mavsdk, const std::string& connection_url);

class Timesync {
public:
    Timesync(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough);

    // void send_timesync_request();
    // [[nodiscard]] int64_t calculate_offset() const;

private:
    int64_t local_time() const;
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

// void Timesync::send_timesync_request() {
//     send_timestamp_ = local_time();
//     mavlink_timesync_t timesync{};
//     timesync.tc1 = 0;
//     timesync.ts1 = send_timestamp_;
//     mavlink_message_t message;
//     mavlink_msg_timesync_pack(
//         mavlink_passthrough_->get_our_sysid(),
//         mavlink_passthrough_->get_our_compid(),
//         &message,
//         timesync.tc1,
//         timesync.ts1,
//         0,
//         0);

//     auto result = mavlink_passthrough_->queue_message([&message](mavsdk::MavlinkAddress, uint8_t) {
//         return message;
//     });
//     if (result != mavsdk::MavlinkPassthrough::Result::Success) {
//         std::cerr << "Failed to send timesync request\n";
//     }
// }

// void Timesync::on_timesync_message(const mavlink_message_t& message) {
//     mavlink_timesync_t timesync;
//     mavlink_msg_timesync_decode(&message, &timesync);

//     if (timesync.tc1 == 0) {
//         mavlink_message_t response;
//         mavlink_msg_timesync_pack(
//             mavlink_passthrough_->get_our_sysid(),
//             mavlink_passthrough_->get_our_compid(),
//             &response,
//             local_time(),
//             timesync.ts1,
//             0,
//             0);

//         auto result = mavlink_passthrough_->queue_message([&response](mavsdk::MavlinkAddress, uint8_t) {
//             return response;
//         });
//         if (result != mavsdk::MavlinkPassthrough::Result::Success) {
//             std::cerr << "Failed to send timesync response\n";
//         }
//     } else {
//         receive_timestamp_ = local_time();
//         offset_ = ((timesync.tc1 - timesync.ts1) + (timesync.ts1 - send_timestamp_)) / 2;
//     }
// }

// [[nodiscard]] int64_t Timesync::calculate_offset() const {
//     return offset_;
// }

int64_t Timesync::local_time() const {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
}

// int main(int argc, char** argv) {
//     if (argc != 2) {
//         std::cerr << "Usage: " << argv[0] << " <connection_url>\n";
//         return 1;
//     }

//     std::string connection_url = argv[1];
//     mavsdk::Mavsdk mavsdk;
//     auto system = connect_to_mavsdk(mavsdk, connection_url);
//     if (!system) {
//         return 1;
//     }

//     auto mavlink_passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(system);

//     Timesync timesync{mavlink_passthrough};

//     std::this_thread::sleep_for(std::chrono::seconds(1));

//     timesync.send_timesync_request();

//     std::this_thread::sleep_for(std::chrono::seconds(2));

//     int64_t offset = timesync.calculate_offset();
//     std::cout << "Estimated offset: " << offset << " nanoseconds\n";

//     return 0;
// }
