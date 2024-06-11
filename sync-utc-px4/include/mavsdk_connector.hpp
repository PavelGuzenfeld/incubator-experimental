#pragma once
#include <mavsdk/mavsdk.h>

std::shared_ptr<mavsdk::System> connect_to_mavsdk(mavsdk::Mavsdk& mavsdk, std::string const& connection_url);
