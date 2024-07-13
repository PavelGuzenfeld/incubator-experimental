#include <mavsdk/mavsdk.h>

std::shared_ptr<mavsdk::System> connect_to_mavsdk(mavsdk::Mavsdk& mavsdk, const std::string& connection_url);
