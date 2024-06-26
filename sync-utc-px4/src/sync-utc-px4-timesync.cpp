#include <cstdint>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <functional>
#include "mavsdk_connector.hpp"

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

    //Test time is actually synchronized

    //subscribe to system time
    mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_SYSTEM_TIME, [](const mavlink_message_t& message) {
            mavlink_system_time_t controller_time;
            mavlink_msg_system_time_decode(&message, &controller_time);
            //Get current utc time in nanoseconds
            auto current_time = std::chrono::high_resolution_clock::now();
            auto offset = controller_time.time_unix_usec - std::chrono::duration_cast<std::chrono::microseconds>(current_time.time_since_epoch()).count();
            std::cout << "Mavlink time utc: " << controller_time.time_unix_usec<< std::endl;
            std::cout << "Mavlink time since boot: " << controller_time.time_boot_ms << std::endl;
            std::cout << "Current time: " << std::chrono::duration_cast<std::chrono::microseconds>(current_time.time_since_epoch()).count() << std::endl;
            std::cout << "Difference: " << offset << std::endl;

        });

    mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_TIMESYNC, [&](const mavlink_message_t& message) {
            mavlink_timesync_t timesync;
            mavlink_msg_timesync_decode(&message, &timesync);

            if (timesync.tc1 == 0) {
                // This is a request, respond back
                mavlink_message_t response;
                auto local_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                mavlink_msg_timesync_pack(
                    mavlink_passthrough->get_our_sysid(),
                    mavlink_passthrough->get_our_compid(),
                    &response,
                    //tc1: time sync counter
                    local_time_us,
                    //ts1: time sync timestamp
                    timesync.ts1,
                    message.sysid,
                    message.compid);

                auto result = mavlink_passthrough->queue_message([message](MavlinkAddress, uint8_t)
                                                      { 
                                                        return message; });
                if (result != mavsdk::MavlinkPassthrough::Result::Success) {
                    std::cerr << "Failed to send timesync response\n";
                }
            } else {
                // This is a response
                // receive_timestamp_ = local_time_ns();
                // offset_ = ((timesync.tc1 - timesync.ts1) + (timesync.ts1 - send_timestamp_)) / 2;
            }
        });

    std::this_thread::sleep_for(std::chrono::seconds(10));

    std::cout<<"Enabling timesync"<<std::endl;
        system->enable_timesync();
    std::this_thread::sleep_for(std::chrono::seconds(30));

    std::cout << "Finished" << std::endl;

    return 0;
}
