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
            //Get current utc time in nanoseconds
            auto current_time = std::chrono::system_clock::now();
            auto offset = controller_time.time_unix_usec - std::chrono::duration_cast<std::chrono::microseconds>(current_time.time_since_epoch()).count();
            std::cout << "Mavlink time utc: " << controller_time.time_unix_usec<< std::endl;
            std::cout << "Mavlink time since boot: " << controller_time.time_boot_ms << std::endl;
            std::cout << "Current time: " << std::chrono::duration_cast<std::chrono::microseconds>(current_time.time_since_epoch()).count() << std::endl;
            std::cout << "Difference: " << offset << std::endl;

        });

    // mavlink_passthrough->subscribe_message(
    //     MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
    //     [](const mavlink_message_t& message) {
    //         mavlink_attitude_quaternion_t attitude_quaternion;
    //         // time_boot_ms	uint32_t	[ms]	Timestamp (time since system boot).
    //         // q1	float	[rad]	Quaternion component 1, w (1 in null-rotation)
    //         // q2	float	[rad]	Quaternion component 2, x (0 in null-rotation)
    //         // q3	float	[rad]	Quaternion component 3, y (0 in null-rotation)
    //         // q4	float	[rad]	Quaternion component 4, z (0 in null-rotation)
    //         // rollspeed	float	[rad/s]	Body frame roll / phi angular speed
    //         // pitchspeed	float	[rad/s]	Body frame pitch / theta angular speed
    //         // yawspeed	float	[rad/s]	Body frame yaw / psi angular speed
    //         mavlink_msg_attitude_quaternion_decode(&message, &attitude_quaternion);
    //         std::cout << "time_boot_ms: " << attitude_quaternion.time_boot_ms << " ms\n"
    //                                     << "q1: " << attitude_quaternion.q1 << '\n' 
    //                                     << "q2: " << attitude_quaternion.q2 << '\n'
    //                                     << "q3: " << attitude_quaternion.q3 << '\n'
    //                                     << "q4: " << attitude_quaternion.q4 << '\n'
    //                                     << "rollspeed: " << attitude_quaternion.rollspeed << " rad/s\n"
    //                                     << "pitchspeed: " << attitude_quaternion.pitchspeed << " rad/s\n"
    //                                     << "yawspeed: " << attitude_quaternion.yawspeed << " rad/s\n";
    //     }
    // );

    mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_SYSTEM_TIME,
        [](const mavlink_message_t& message) {
            mavlink_system_time_t system_time;
            mavlink_msg_system_time_decode(&message, &system_time);
            //time_unix_usec	uint64_t	[us]	Time since system start
            //time_boot_ms	uint32_t	[ms]	Time since system boot
            

            auto px4_time_us = system_time.time_unix_usec;
            auto px4_time = std::chrono::high_resolution_clock::time_point(std::chrono::microseconds(px4_time_us));
            auto local_time = std::chrono::high_resolution_clock::now();

            auto time_diff_us = std::chrono::duration_cast<std::chrono::microseconds>(local_time - px4_time).count();
            std::cout << "Time difference: " << time_diff_us << " µs\n";

            const int64_t desired_precision_us = 1000; // 1 millisecond in microseconds
            if (std::abs(time_diff_us) > desired_precision_us) {
                std::cerr << "Warning: Time difference is greater than 1 millisecond (" << desired_precision_us << " µs)!\n";
            } else {
                std::cout << "Times are synchronized within 1 millisecond (" << desired_precision_us << " µs).\n";
            }
        });

    //sleep for 20 sec
    std::cout << "Sleeping for 20 seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(20));


    return 0;
}
