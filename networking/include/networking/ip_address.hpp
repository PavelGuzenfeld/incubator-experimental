#include "fmt/compile.h" // Use compile-time formatting
#include <compare>       // For <=>
#include <cstdint>       // For uint32_t, uint8_t
#include <stdexcept>     // For std::domain_error
#include <string>        // For std::string
#include <string_view>   // For std::string_view

/**
 * @brief A strongly-typed, validated subnet mask.
 *
 * Ensures the mask value is always between 0 and 32.
 */
class Mask
{
public:
    /**
     * @brief Constructs a Mask, validating the value.
     * @throws std::domain_error if mask > 32.
     */
    constexpr explicit Mask(uint8_t mask) : mask_(mask)
    {
        if (mask > 32)
        {
            throw std::domain_error("Mask value must be between 0 and 32");
        }
    }

    /**
     * @brief Gets the raw integer value of the mask.
     */
    constexpr uint8_t as_int() const { return mask_; }

    constexpr uint64_t get_subnet_cardinality()
    {
        if (mask_ == 32)
            return 1;
        if (mask_ == 31)
            return 2;
        uint8_t host_bits = 32 - mask_;
        return (1ULL << host_bits) - 2;
    }

private:
    uint8_t mask_;
};

/**
 * @brief A class to represent and manipulate an IPv4 address.
 *
 * Stores the IP as a uint32_t and provides methods for
 * subnet calculations and formatting.
 */
class IpAddress
{
public:
    /**
     * @brief Construct from a 32-bit integer.
     */
    constexpr explicit IpAddress(uint32_t ip) : ip_int_(ip) {}

    /**
     * @brief Construct from a string, validating and parsing it.
     * @throws std::domain_error if the string is not a valid IP.
     */
    constexpr explicit IpAddress(std::string_view ip_str)
        : ip_int_(get_ip_integral_equivalent(ip_str)) {}

    /**
     * @brief Get the raw 32-bit integer value.
     */
    constexpr uint32_t as_int() const { return ip_int_; }

    /**
     * @brief Get the "A.B.C.D" string format.
     */
    std::string to_string() const
    {
        // FMT_COMPILE pre-parses the format string at compile time.
        return fmt::format(FMT_COMPILE("{}.{}.{}.{}"), (ip_int_ >> 24) & 0xFF,
                           (ip_int_ >> 16) & 0xFF, (ip_int_ >> 8) & 0xFF,
                           ip_int_ & 0xFF);
    }

    /**
     * @brief Get the broadcast address for this IP given a mask.
     */
    constexpr IpAddress get_broadcast_address(Mask mask) const
    {
        uint32_t subnet_mask_int =
            (mask.as_int() == 0) ? 0U : (0xFFFFFFFFU << (32 - mask.as_int()));
        uint32_t host_mask_int = ~subnet_mask_int;
        //
        return IpAddress(ip_int_ | host_mask_int);
    }

    /**
     * @brief Get the network ID for this IP given a mask.
     */
    constexpr IpAddress get_network_id(Mask mask) const
    {
        uint32_t subnet_mask_int =
            (mask.as_int() == 0) ? 0U : (0xFFFFFFFFU << (32 - mask.as_int()));
        //
        return IpAddress(ip_int_ & subnet_mask_int);
    }

    /**
     * @brief Check if this IP belongs to a given subnet.
     */
    constexpr bool is_in_subnet(IpAddress network_id, Mask mask) const
    {
        // An IP is in the subnet if its calculated network ID
        // matches the provided network_id.
        return this->get_network_id(mask).as_int() == network_id.as_int();
    }

    // --- Comparison Operators ---
    constexpr auto operator<=>(const IpAddress &other) const = default;
    constexpr bool operator==(const IpAddress &other) const = default;

private:
    /**
     * @brief Validating parser for IP strings.
     */
    constexpr uint32_t get_ip_integral_equivalent(std::string_view ip_address)
    {
        uint32_t result = 0;
        uint32_t current_octet = 0;
        int octet_count = 0;
        bool octet_has_value = false;

        for (const char c : ip_address)
        {
            if (c == '.')
            {
                if (!octet_has_value)
                {
                    throw std::domain_error("Invalid IP format: empty octet");
                }
                if (current_octet > 255)
                {
                    throw std::domain_error("Invalid IP octet value > 255");
                }
                result = (result << 8) | current_octet;
                current_octet = 0;
                octet_count++;
                octet_has_value = false;
            }
            else if (c >= '0' && c <= '9')
            {
                current_octet = (current_octet * 10) + (c - '0');
                octet_has_value = true;
            }
            else
            {
                throw std::domain_error("Invalid character in IP address");
            }
        }

        // Check the last octet
        if (!octet_has_value)
        {
            throw std::domain_error("Invalid IP format: empty last octet");
        }
        if (current_octet > 255)
        {
            throw std::domain_error("Invalid IP octet value > 255");
        }
        result = (result << 8) | current_octet;
        octet_count++;

        if (octet_count != 4)
        {
            throw std::domain_error("Invalid IP format: must have 4 octets");
        }
        return result;
    }

private:
    uint32_t ip_int_;
};
