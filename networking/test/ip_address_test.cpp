#include "networking/ip_address.hpp"
#include <cassert> // For assert() in runtime tests

/*
 * ===================================================================
 * COMPILE-TIME TESTS (with static_assert)
 * ===================================================================
 */
int main()
{
    using namespace std::literals; // Use string_view literals (e.g.,
                                   // "1.2.3.4"sv)

    // --- Tests for get_broadcast_address ---
    static_assert(IpAddress("192.168.2.10"sv).get_broadcast_address(Mask(24)) ==
                      IpAddress("192.168.2.255"sv),
                  "Q1 Test 1 failed");
    static_assert(IpAddress("10.1.23.10"sv).get_broadcast_address(Mask(20)) ==
                      IpAddress("10.1.31.255"sv),
                  "Q1 Test 2 failed");

    // --- Tests for get_ip_integral_equivalent (via constructor) ---
    static_assert(IpAddress("192.168.2.10"sv).as_int() == 3232236042U,
                  "Q2 Test 1 failed");
    static_assert(IpAddress("10.1.23.10"sv).as_int() == 167843594U,
                  "Q2 Test 2 failed");

    // --- Tests for get_network_id ---
    static_assert(IpAddress("192.168.2.10"sv).get_network_id(Mask(20)) ==
                      IpAddress("192.168.0.0"sv),
                  "Q4 Test 1 failed");
    static_assert(IpAddress("10.1.23.10"sv).get_network_id(Mask(20)) ==
                      IpAddress("10.1.16.0"sv),
                  "Q4 Test 2 failed");

    // --- Tests for get_subnet_cardinality ---
    static_assert((Mask(24).get_subnet_cardinality()) == 254,
                  "Q5 Test 1 failed");
    static_assert((Mask(16).get_subnet_cardinality()) == 65534,
                  "Q5 Test 4 failed");

    // --- Tests for is_in_subnet ---
    static_assert(IpAddress("192.168.0.13"sv)
                      .is_in_subnet(IpAddress("192.168.0.0"sv), Mask(24)),
                  "Q6 Test 1 failed");
    static_assert(!IpAddress("192.168.1.13"sv)
                       .is_in_subnet(IpAddress("192.168.0.0"sv), Mask(24)),
                  "Q6 Test 2 failed");

    fmt::print("--- All compile-time tests passed! ---\n");
    fmt::print("--- Running runtime tests: ---\n");

    std::string bc1 =
        IpAddress("192.168.2.10"sv).get_broadcast_address(Mask(24)).to_string();
    fmt::print("Q1 Test: {}\n", bc1);
    assert(bc1 == "192.168.2.255"sv);

    std::string ip_str1 = IpAddress(2058138165U).to_string();
    fmt::print("Q3 Test: {}\n", ip_str1);
    assert(ip_str1 == "122.172.178.53"sv);

    std::string ip_str2 = IpAddress(3232236032U).to_string();
    fmt::print("Q3 Test: {}\n", ip_str2);
    assert(ip_str2 == "192.168.2.0"sv);

    std::string net_id1 =
        IpAddress("192.168.2.10"sv).get_network_id(Mask(20)).to_string();
    fmt::print("Q4 Test: {}\n", net_id1);
    assert(net_id1 == "192.168.0.0"sv);

    uint64_t card = Mask(24).get_subnet_cardinality();
    fmt::print("Q5 Test: {}\n", card);
    assert(card == 254);

    bool res = IpAddress("192.168.0.13"sv)
                   .is_in_subnet(IpAddress("192.168.0.0"sv), Mask(24));
    fmt::print("Q6 Test: {}\n", res);
    assert(res == true);

    try
    {
        IpAddress("192.168.1.256"sv); // Octet > 255
        assert(false && "Validation failed to catch octet > 255");
    }
    catch (const std::domain_error &e)
    {
        fmt::print("Validation Test (octet > 255): Passed ({})\n", e.what());
    }

    try
    {
        IpAddress("192.168.1.a"sv); // Invalid char
        assert(false && "Validation failed to catch invalid char");
    }
    catch (const std::domain_error &e)
    {
        fmt::print("Validation Test (invalid char): Passed ({})\n", e.what());
    }

    try
    {
        IpAddress("192.168.1"sv); // Not 4 octets
        assert(false && "Validation failed to catch wrong octet count");
    }
    catch (const std::domain_error &e)
    {
        fmt::print("Validation Test (octet count): Passed ({})\n", e.what());
    }

    try
    {
        Mask(33); // Invalid mask
        assert(false && "Validation failed to catch invalid mask");
    }
    catch (const std::domain_error &e)
    {
        fmt::print("Validation Test (mask > 32): Passed ({})\n", e.what());
    }

    fmt::print("--- All runtime tests passed! ---\n");

    return 0;
}