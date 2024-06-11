#include "mavsdk_connector.hpp"
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

void set_system_time(time_t sec, suseconds_t usec) {
    struct timeval tv;
    tv.tv_sec = sec;
    tv.tv_usec = usec;
    if (settimeofday(&tv, nullptr) != 0) {
        perror("settimeofday");
        throw std::runtime_error("Failed to set system time.");
    }
}

void update_system_time(const mavlink_system_time_t& system_time) {
    auto px4_time_us = system_time.time_unix_usec;
    auto px4_time_ns = px4_time_us * 1000; // Convert microseconds to nanoseconds

    // Current system time
    auto now = std::chrono::high_resolution_clock::now();
    auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    // Calculate the time offset
    auto offset_ns = px4_time_ns - now_ns;
    auto offset_us = offset_ns / 1000;

    std::cout << "Time offset: " << offset_ns << " ns (" << offset_us << " µs)\n";

    // Calculate new system time
    auto new_system_time = now + std::chrono::nanoseconds(offset_ns);
    auto new_system_time_sec = std::chrono::time_point_cast<std::chrono::seconds>(new_system_time).time_since_epoch().count();
    auto new_system_time_usec = std::chrono::duration_cast<std::chrono::microseconds>(new_system_time.time_since_epoch()).count() % 1000000;

    // Set the system time
    set_system_time(new_system_time_sec, new_system_time_usec);

    std::cout << "System time updated based on PX4 time.\n";
}

void compare_system_times(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough) {
    mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_SYSTEM_TIME,
        [=](const mavlink_message_t& message) {
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
                // send_system_time(mavlink_passthrough);
                update_system_time(system_time);
            } else {
                std::cout << "Times are synchronized within 1 millisecond (" << desired_precision_us << " µs).\n";
            }
        }
    );
}


void print_attitude_quaternion(std::shared_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough) {
    mavlink_passthrough->subscribe_message(
        MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
        [](const mavlink_message_t& message) {
            mavlink_attitude_quaternion_t attitude_quaternion;
            // time_boot_ms	uint32_t	[ms]	Timestamp (time since system boot).
            // q1	float	[rad]	Quaternion component 1, w (1 in null-rotation)
            // q2	float	[rad]	Quaternion component 2, x (0 in null-rotation)
            // q3	float	[rad]	Quaternion component 3, y (0 in null-rotation)
            // q4	float	[rad]	Quaternion component 4, z (0 in null-rotation)
            // rollspeed	float	[rad/s]	Body frame roll / phi angular speed
            // pitchspeed	float	[rad/s]	Body frame pitch / theta angular speed
            // yawspeed	float	[rad/s]	Body frame yaw / psi angular speed
            mavlink_msg_attitude_quaternion_decode(&message, &attitude_quaternion);
            std::cout << "time_boot_ms: " << attitude_quaternion.time_boot_ms << " ms\n"
                                        << "q1: " << attitude_quaternion.q1 << '\n' 
                                        << "q2: " << attitude_quaternion.q2 << '\n'
                                        << "q3: " << attitude_quaternion.q3 << '\n'
                                        << "q4: " << attitude_quaternion.q4 << '\n'
                                        << "rollspeed: " << attitude_quaternion.rollspeed << " rad/s\n"
                                        << "pitchspeed: " << attitude_quaternion.pitchspeed << " rad/s\n"
                                        << "yawspeed: " << attitude_quaternion.yawspeed << " rad/s\n";
        }
    );
}

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

    // std::cout << "Sending time sync request to match PX4 time with offboard computer time..." << std::endl;
    // send_time_sync_request(mavlink_passthrough);

    std::cout << "Listening for PX4 time sync response to compare with local system time..." << std::endl;
    compare_system_times(mavlink_passthrough);
    print_attitude_quaternion(mavlink_passthrough);

    std::this_thread::sleep_for(std::chrono::seconds(10)); // Keep the program running for a while to receive messages

    return 0;
}


