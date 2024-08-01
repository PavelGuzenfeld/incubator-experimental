#include "mavsdk_connector.hpp"
#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <mavsdk/mavsdk.h>
#include <thread>

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <connection_url>\n";
        return 1;
    }

    std::string connection_url = argv[1];
    mavsdk::Mavsdk mavsdk{mavsdk::Mavsdk::Configuration(mavsdk::Mavsdk::ComponentType::CompanionComputer)};
    auto systems = connect_to_mavsdk(mavsdk, connection_url);
    if (systems.empty())
    {
        std::cerr << "Failed to connect to any system.\n";
        return 1;
    }

    std::cout << "Connected to " << systems.size() << " system(s)." << std::endl;

    // Keep the program running for additional connections
    std::cout << "Waiting for additional connections...\n";
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(120));
    }

    return 0;
}