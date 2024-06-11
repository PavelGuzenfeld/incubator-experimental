#include <mavsdk/mavsdk.h>
#include <chrono>
#include <future>

std::shared_ptr<mavsdk::System> connect_to_mavsdk(mavsdk::Mavsdk& mavsdk, std::string const& connection_url)
 {
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);
    if (connection_result != mavsdk::ConnectionResult::Success) {
        return nullptr;
    }

    auto prom = std::promise<std::shared_ptr<mavsdk::System>>();
    auto fut = prom.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &prom]() {
        auto system = mavsdk.systems().back();
        if (system->is_connected()) {
            prom.set_value(system);
        }
    });

    if (fut.wait_for(std::chrono::seconds(10)) == std::future_status::timeout) {
        return nullptr;
    }

    return fut.get();
}