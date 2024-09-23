#include <functional>
#include <iostream>
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <variant>
#include <vector>

// Multi-type variant for observations
using EventVariant = std::variant<int, float, std::string>;

// Observer Interface
class Observer
{
public:
    virtual void update(const EventVariant &value) = 0;
    virtual ~Observer() = default;
};

// Observable with type-dispatching
class Observable
{
public:
    // Register observer for a specific type
    template <typename T>
    void addObserver(std::shared_ptr<Observer> observer)
    {
        observers_[std::type_index(typeid(T))].push_back(observer);
    }

    // Remove observer for a specific type
    template <typename T>
    void removeObserver(std::shared_ptr<Observer> observer)
    {
        auto &obsList = observers_[std::type_index(typeid(T))];
        obsList.erase(std::remove(obsList.begin(), obsList.end(), observer), obsList.end());
    }

    // Notify observers of a specific type
    template <typename T>
    void notify(const T &value)
    {
        std::type_index type = std::type_index(typeid(T));
        if (observers_.find(type) != observers_.end())
        {
            for (auto &observer : observers_[type])
            {
                observer->update(value);
            }
        }
    }

private:
    std::unordered_map<std::type_index, std::vector<std::shared_ptr<Observer>>> observers_;
};

// Concrete Observer using std::variant
class ConsoleObserver : public Observer
{
public:
    void update(const EventVariant &value) override
    {
        std::visit([this](auto &&arg)
                   { handleUpdate(arg); }, value);
    }

private:
    // Handle different types
    void handleUpdate(int value)
    {
        std::cout << "Received int: " << value << std::endl;
    }

    void handleUpdate(float value)
    {
        std::cout << "Received float: " << value << std::endl;
    }

    void handleUpdate(const std::string &value)
    {
        std::cout << "Received string: " << value << std::endl;
    }
};

// Example
int main()
{
    auto observable = std::make_shared<Observable>();

    auto observer1 = std::make_shared<ConsoleObserver>();

    // Register observer for specific types
    observable->addObserver<int>(observer1);
    observable->addObserver<float>(observer1);

    // Notify observers
    observable->notify(42);    // Observer for int
    observable->notify(3.14f); // Observer for float
    observable->notify(std::string("No observer for this"));

    return 0;
}
