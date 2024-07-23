#include "cyclic_view/cyclic_view.hpp"
#include "mavsdk_connector.hpp"
#include <chrono>
#include <concepts>
#include <cstdint>
#include <functional>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <memory>
#include <numeric>
#include <ranges>
#include <thread>
#include <type_traits>
#include <utility>

template<typename T>
concept Arithmetic = std::is_arithmetic_v<T>;

template<typename T>
concept ArithmeticPair = requires(T a) {
    { std::get<0>(a) } -> std::convertible_to<Arithmetic>;
    { std::get<1>(a) } -> std::convertible_to<Arithmetic>;
};

template<typename T>
concept ArithmeticPairCollection = requires(T a) {
    { a.begin() } -> std::input_iterator;
    { a.end() } -> std::input_iterator;
    requires ArithmeticPair<typename T::value_type>;
};

constexpr auto calculate_average_difference(const ArithmeticPairCollection auto& collection) {
    return std::transform_reduce(collection.begin(), collection.end(), 0.0, std::plus<>(), [](const auto& pair) {
        auto [first, second] = pair;
        return first - second;
    }) / static_cast<double>(collection.size());
}


auto time_collection = std::array<std::tuple<int64_t, int64_t>, 200>{};
auto time_collection_view = cyclic::CyclicView(time_collection, 200);

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <connection_url>\n";
        return 1;
    }

    std::string connection_url = argv[1];
    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration(mavsdk::Mavsdk::ComponentType::CompanionComputer)};
    auto system = connect_to_mavsdk(mavsdk, connection_url);
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
            auto mavlink_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::microseconds(controller_time.time_unix_usec));
            auto mavlink_boot_time_ms = std::chrono::milliseconds(controller_time.time_boot_ms);
            auto diff = system_time_ms - mavlink_time_ms;

            std::cout << "Mavlink time: " << mavlink_time_ms.count() << " ms\n";
            std::cout << "Current system time: " << system_time_ms.count() << " ms\n";
            std::cout << "Current steady time: " << steady_time_ms.count() << " ms\n";
            std::cout << "Difference: " << diff.count() << " ms\n";
            std::cout << "Mavlink time since boot: " << mavlink_boot_time_ms.count() << " ms\n";
            std::cout << "-------------------\n";

            // insert_and_limit(time_collection, std::make_tuple(steady_time_ms.count(), mavlink_boot_time_ms.count()), 200);
            time_collection_view.push(std::make_tuple(steady_time_ms.count(), mavlink_boot_time_ms.count()));

            double avg_diff = calculate_average_difference(time_collection);
            std::cout << "Average difference: " << avg_diff << " ms\n";
        });

    std::cout << "Sleeping for 20 seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(20));

    return 0;
}
