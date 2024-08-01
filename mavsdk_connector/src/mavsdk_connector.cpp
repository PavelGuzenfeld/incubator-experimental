#include <mavsdk/mavsdk.h>
#include <chrono>
#include <future>

std::vector<std::shared_ptr<mavsdk::System>> connect_to_mavsdk(mavsdk::Mavsdk &mavsdk, const std::string &connection_url)
{
    mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);
    if (connection_result != mavsdk::ConnectionResult::Success)
    {
        return {};
    }

    std::vector<std::shared_ptr<mavsdk::System>> connected_systems;
    std::mutex mutex;
    std::condition_variable cv;
    bool system_connected = false;

    mavsdk.subscribe_on_new_system([&mavsdk, &connected_systems, &mutex, &cv, &system_connected]()
                                   {
        auto system = mavsdk.systems().back();
        if (system->is_connected()) {
            std::lock_guard<std::mutex> lock(mutex);
            connected_systems.push_back(system);
            system_connected = true;
            cv.notify_one();
        } });

    {
        std::unique_lock<std::mutex> lock(mutex);
        if (!cv.wait_for(lock, std::chrono::seconds(10), [&system_connected]
                         { return system_connected; }))
        {
            // Timeout
            return {};
        }
    }

    return connected_systems;
}