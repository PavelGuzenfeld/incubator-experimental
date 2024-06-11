#include <iostream>
#include <memory>

// Forward declarations
class ServiceOne;
class ServiceTwo;

// Template interface IService
template <typename T>
struct IService
{
    virtual ~IService() = default;
    virtual void doSomething() const = 0;

    // Static method to create an instance of T
    static std::shared_ptr<IService> create()
    {
        return std::make_shared<T>();
    }
};

// First service implementation
class ServiceOne : public IService<ServiceOne>
{
public:
    void doSomething() const override
    {
        std::cout << "Service One doing something\n";
    }
};

// Second service implementation
class ServiceTwo : public IService<ServiceTwo>
{
public:
    void doSomething() const override
    {
        std::cout << "Service Two doing something else\n";
    }
};

// Client code with a template parameter
template <typename ServiceType>
void clientCode(const std::shared_ptr<IService<ServiceType>> &service)
{
    service->doSomething();
}

// Main function
int main()
{
    auto serviceOne = IService<ServiceOne>::create();
    auto serviceTwo = IService<ServiceTwo>::create();

    clientCode(serviceOne);
    clientCode(serviceTwo);

    return 0;
}
