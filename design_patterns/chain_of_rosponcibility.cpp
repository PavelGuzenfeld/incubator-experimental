#include <iostream>
#include <tuple>
#include <utility>

// Base case: When no handlers are left in the chain, the request is unhandled.
template <typename Request>
void handleRequest(const Request &request)
{
    std::cout << "Request unhandled: " << request << std::endl;
}

// Recursive template to go through each handler in the chain.
template <typename Request, typename FirstHandler, typename... RestHandlers>
void handleRequest(const Request &request, FirstHandler &&firstHandler, RestHandlers &&...restHandlers)
{
    // If the first handler handles the request, stop the chain.
    if (firstHandler(request))
    {
        return;
    }
    // Otherwise, pass the request to the next handler in the chain.
    handleRequest(request, std::forward<RestHandlers>(restHandlers)...);
}

// Broker class to manage the chain of responsibility for the same type of request.
template <typename Request, typename... Handlers>
class Broker
{
public:
    // Ensure the constructor accepts handler arguments
    template <typename... HandlerArgs>
    explicit Broker(HandlerArgs &&...handlers)
        : handlers_(std::forward<HandlerArgs>(handlers)...) {}

    void processRequest(const Request &request)
    {
        // Call the recursive handleRequest function, passing all handlers.
        std::apply([&](auto &&...handlers)
                   { handleRequest(request, handlers...); }, handlers_);
    }

private:
    std::tuple<Handlers...> handlers_; // Store the chain of handlers in a tuple.
};

int main()
{
    // Define lambdas to handle the same type of request (int in this case).
    auto handleEven = [](int request)
    {
        if (request % 2 == 0)
        {
            std::cout << "Handled even number: " << request << std::endl;
            return true; // Handled successfully.
        }
        return false; // Pass to the next handler.
    };

    auto handleDivisibleByThree = [](int request)
    {
        if (request % 3 == 0)
        {
            std::cout << "Handled divisible by 3: " << request << std::endl;
            return true; // Handled successfully.
        }
        return false; // Pass to the next handler.
    };

    auto handleGreaterThanTen = [](int request)
    {
        if (request > 10)
        {
            std::cout << "Handled greater than 10: " << request << std::endl;
            return true;
        }
        return false; // Pass to the next handler.
    };

    auto handleDefault = [](int request)
    {
        std::cout << "Default handler for request: " << request << std::endl;
        return true; // Default handler always handles the request.
    };

    // Create a broker for processing the same type of request (int).
    Broker<int, decltype(handleEven), decltype(handleDivisibleByThree), decltype(handleGreaterThanTen), decltype(handleDefault)>
        broker(handleEven, handleDivisibleByThree, handleGreaterThanTen, handleDefault);

    // Test different requests through the chain.
    broker.processRequest(4);  // Handled by handleEven
    broker.processRequest(9);  // Handled by handleDivisibleByThree
    broker.processRequest(11); // Handled by handleGreaterThanTen
    broker.processRequest(5);  // Default handler handles it

    return 0;
}
