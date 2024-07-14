#include <cstdint>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <chrono>
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

    system->enable_timesync();

    auto mavlink_passthrough = std::make_shared<mavsdk::MavlinkPassthrough>(system);

    mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_SYSTEM_TIME, [](const mavlink_message_t& message) {
            mavlink_system_time_t controller_time;
            mavlink_msg_system_time_decode(&message, &controller_time);
            auto system_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
            auto steady_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch());
            auto mavlink_time_ms =  std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::microseconds(controller_time.time_unix_usec));
            auto mavlink_boot_time_ms =  std::chrono::milliseconds(controller_time.time_boot_ms);
            auto diff = system_time_ms - mavlink_time_ms;
            std::cout << "Mavlink time: " << mavlink_time_ms << "\n";
            std::cout << "Current system time: " << system_time_ms << "\n";
            std::cout << "Current steady time: " << steady_time_ms << "\n";
            std::cout << "Difference: " << diff << "\n";
            std::cout << "Mavlink time since boot: " << mavlink_boot_time_ms << "\n";
        });

    //sleep for 20 sec
    std::cout << "Sleeping for 20 seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(20));


    return 0;
}
