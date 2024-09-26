#include <iostream>
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <type_traits>
#include <cstring>
#include <fmt/core.h>
#include <compare>

// Template class with an optional description argument
template <typename T, const char* Description, std::size_t Align = alignof(T)>
class PhysicalType {
public:
    constexpr PhysicalType(T value = 0) : value_(value) {}

    // Arithmetic operations
    constexpr PhysicalType operator+(const PhysicalType& other) const noexcept {
        return PhysicalType(value_ + other.value_);
    }

    constexpr PhysicalType operator-(const PhysicalType& other) const noexcept {
        return PhysicalType(value_ - other.value_);
    }

    constexpr PhysicalType operator*(const PhysicalType& other) const noexcept {
        return PhysicalType(value_ * other.value_);
    }

    constexpr PhysicalType operator/(const PhysicalType& other) const noexcept {
        return PhysicalType(value_ / other.value_);
    }

    constexpr PhysicalType operator%(const PhysicalType& other) const noexcept {
        return PhysicalType(value_ % other.value_);
    }

    // Spaceship operator for comparison
    auto operator<=>(const PhysicalType&) const = default;

    // Bitwise operations
    constexpr PhysicalType operator&(const PhysicalType& other) const noexcept {
        return PhysicalType(value_ & other.value_);
    }

    constexpr PhysicalType operator|(const PhysicalType& other) const noexcept {
        return PhysicalType(value_ | other.value_);
    }

    constexpr PhysicalType operator^(const PhysicalType& other) const noexcept {
        return PhysicalType(value_ ^ other.value_);
    }

    constexpr PhysicalType operator~() const noexcept {
        return PhysicalType(~value_);
    }

    constexpr PhysicalType operator<<(std::size_t shift) const noexcept {
        return PhysicalType(value_ << shift);
    }

    constexpr PhysicalType operator>>(std::size_t shift) const noexcept {
        return PhysicalType(value_ >> shift);
    }

    // Serialization and deserialization
    template <typename Buffer>
    void serialize(Buffer& buffer) const {
        const char* data = reinterpret_cast<const char*>(&value_);
        buffer.insert(buffer.end(), data, data + sizeof(value_));
    }

    template <typename Buffer>
    static PhysicalType deserialize(const Buffer& buffer, std::size_t& offset) {
        if (offset + sizeof(T) > buffer.size()) {
            throw std::runtime_error("Buffer too small for deserialization");
        }
        T value;
        std::memcpy(&value, buffer.data() + offset, sizeof(T));
        offset += sizeof(T);
        return PhysicalType(value);
    }

    [[nodiscard]] constexpr T value() const noexcept { return value_; }

    // Static method to return the type description (metadata)
    static const char* description() {
        return Description;
    }

private:
    alignas(Align) T value_;
};

// Static strings for descriptions
constexpr char meter_sign[]= "m";
constexpr char kilogram_desc[] = "Kilogram";
constexpr char speed_desc[] = "Speed";
constexpr char bool_desc[] = "Boolean Flag"; // Description for logical_flag

// Typedefs for physical units with template description

using meter = PhysicalType<uint64_t, meter_sign>;
using kilogram = PhysicalType<uint32_t, kilogram_desc>;
using speed = PhysicalType<double, speed_desc>;
using logical_flag = PhysicalType<bool, bool_desc>;  // Add description for logical_flag

// Force the compiler to avoid padding using #pragma pack
#pragma pack(push, 1)
struct MixedData {
    meter distance;
    kilogram mass;
    speed velocity;
    logical_flag is_active;
    logical_flag another;

    void print() const {
        fmt::print("Distance: {} ({}), Mass: {} ({}), Velocity: {} ({}), Is Active: {} ({}), Another: {} ({}).\n", 
                   distance.value(), meter::description(),
                   mass.value(), kilogram::description(),
                   velocity.value(), speed::description(),
                   is_active.value(), logical_flag::description(),
                   another.value(), logical_flag::description());
    }
};
#pragma pack(pop)

int main() {
    MixedData data{
        meter(10000),           // 10000 meters
        kilogram(75),           // 75 kilograms
        speed(55.5),            // 55.5 meters/second
        logical_flag(true),      // Is Active: true
        logical_flag(false)     // Another: false
    };

    data.print();

    // Check sizes of individual types
    fmt::print("Size of meter: {} bytes\n", sizeof(meter));
    fmt::print("Size of kilogram: {} bytes\n", sizeof(kilogram));
    fmt::print("Size of speed: {} bytes\n", sizeof(speed));
    fmt::print("Size of logical_flag (bool): {} bytes\n", sizeof(logical_flag));

    // Check total size of MixedData
    fmt::print("Total size of MixedData: {} bytes\n", sizeof(MixedData));

    // Manually calculate expected size (without padding):
    std::size_t expected_size = sizeof(meter) + sizeof(kilogram) + sizeof(speed) + sizeof(logical_flag) + sizeof(logical_flag);
    fmt::print("Expected size (without padding): {} bytes\n", expected_size);

    return 0;
}
