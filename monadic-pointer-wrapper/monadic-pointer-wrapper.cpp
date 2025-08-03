#include <concepts>
#include <fmt/core.h>
#include <map>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

// --- Type Traits from Slides ---
template <typename T>
concept cIsPointer = std::is_pointer_v<T>;

// --- CPtr Monadic Wrapper (from slides) ---
template <cIsPointer TPtr>
struct CPtr
{
private:
    TPtr m_Ptr{};

public:
    CPtr() = default;
    explicit CPtr(TPtr ptr) : m_Ptr(ptr) {}

    explicit operator bool() const { return m_Ptr != nullptr; }

    // Monadic 'bind': chain operations that return pointers
    template <typename TCall>
    auto and_then(TCall &&fInvoke)
    {
        using TRef = std::add_lvalue_reference_t<std::remove_pointer_t<TPtr>>;
        using TRet = std::invoke_result_t<TCall, TRef>;

        if constexpr (cIsPointer<TRet>)
        {
            if (m_Ptr)
            {
                return CPtr<TRet>(std::invoke(std::forward<TCall>(fInvoke), *m_Ptr));
            }
            return CPtr<TRet>{};
        }
        else
        {
            static_assert(!std::is_same_v<TRet, TRet>, "and_then function must return a pointer type.");
        }
    }

    // Monadic 'fmap': apply a function to the value and wrap in std::optional
    template <typename TCall>
    auto transform(TCall &&fInvoke)
    {
        using TRef = std::add_lvalue_reference_t<std::remove_pointer_t<TPtr>>;
        using TRet = std::invoke_result_t<TCall, TRef>;

        if (m_Ptr)
        {
            return std::optional<TRet>(std::invoke(std::forward<TCall>(fInvoke), *m_Ptr));
        }
        return std::optional<TRet>{};
    }
};

// --- Mock Database Structures and Data (as seen in slides) ---

struct Key
{
    int id;
    // ✅ CORRECTION: The spaceship operator must be defined inside the class/struct.
    auto operator<=>(const Key &) const = default;
};
struct CLocation
{
    int row, col;
    // ✅ CORRECTION: The spaceship operator must be defined inside the class/struct.
    auto operator<=>(const CLocation &) const = default;
};

struct NumericCell
{
    int value;
};
using TableData = std::map<CLocation, NumericCell>;
using Element = std::map<std::string, TableData>;
struct CDb
{
    std::map<Key, Element> elements;
};

// --- Mock Database Access Functions ---

Element *getElementPtr(CDb &db, const Key &key)
{
    auto it = db.elements.find(key);
    return (it != db.elements.end()) ? &it->second : nullptr;
}

TableData *getTablePtr(Element &element, const std::string &tableName)
{
    auto it = element.find(tableName);
    return (it != element.end()) ? &it->second : nullptr;
}

NumericCell *getCellPtr(TableData &table, const CLocation &loc)
{
    auto it = table.find(loc);
    return (it != table.end()) ? &it->second : nullptr;
}

// Last function in the pointer chain, returns a pointer to the raw value
int *getNumericCellValue(NumericCell &cell)
{
    return &cell.value;
}

// --- Main Usage Example from Slides ---

std::optional<bool> getIntCellValueNegative(CDb &db, const Key &key, const std::string &tableName, const CLocation &cellLocation)
{
    return CPtr(&db)
        .and_then([&](CDb &d)
                  { return getElementPtr(d, key); })
        .and_then([&](Element &e)
                  { return getTablePtr(e, tableName); })
        .and_then([&](TableData &t)
                  { return getCellPtr(t, cellLocation); })
        .and_then([](NumericCell &c)
                  { return getNumericCellValue(c); })
        .transform([](int &val)
                   { return val < 0; });
}

int main()
{
    // 1. Setup mock database with data
    CDb database{
        .elements = {
            {Key{42}, {{"user_data", {
                                         {CLocation{1, 1}, NumericCell{-99}},
                                         {CLocation{1, 2}, NumericCell{150}},
                                     }},
                       {"system_logs", {
                                           {CLocation{5, 8}, NumericCell{-1}},
                                       }}}}}};

    // --- 2. Successful chain: find a negative value ---
    fmt::print("## Use Case 1: Successful search ##\n");
    auto key_success = Key{42};
    auto loc_success = CLocation{1, 1};
    fmt::print("Searching for Key ID: {}, Location: ({},{})\n", key_success.id, loc_success.row, loc_success.col);

    if (auto result = getIntCellValueNegative(database, key_success, "user_data", loc_success))
    {
        fmt::print("Result: Found value. Is it negative? {}\n", *result);
    }
    else
    {
        fmt::print("Result: Chain failed, value not found.\n");
    }

    // --- 3. Failed chain: table name does not exist ---
    fmt::print("\n## Use Case 2: Failing search (bad table name) ##\n");
    auto key_fail = Key{42};
    auto loc_fail = CLocation{1, 1};
    fmt::print("Searching for Key ID: {}, Location: ({},{})\n", key_fail.id, loc_fail.row, loc_fail.col);

    if (auto result = getIntCellValueNegative(database, key_fail, "nonexistent_table", loc_fail))
    {
        fmt::print("Result: Found value. Is it negative? {}\n", *result);
    }
    else
    {
        fmt::print("Result: Chain failed, value not found.\n");
    }
}