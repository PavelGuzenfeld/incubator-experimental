#include <iostream>
#include <iterator>
#include <vector>

int main()
{
    // Vector example
    {

        // Create a vector of integers
        std::vector<int> numbers = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

        // Example using std::next
        auto it_next = std::next(numbers.begin(), 3); // Advance iterator by 3 positions
        std::cout << "Element at position 3 (using std::next): " << *it_next << std::endl;

        // Example using std::prev
        auto it_prev = std::prev(numbers.end(), 3); // Move iterator back by 3 positions from end
        std::cout << "Element 3 positions before end (using std::prev): " << *it_prev << std::endl;

        // Example using std::advance
        auto it_advance = numbers.begin();
        std::advance(it_advance, 5); // Advance iterator by 5 positions
        std::cout << "Element at position 5 (using std::advance): " << *it_advance << std::endl;

        // Example using std::distance
        auto it_start = numbers.begin();
        auto it_end = numbers.end();
        auto dist = std::distance(it_start, it_end); // Get the number of elements between two iterators
        std::cout << "Distance from begin to end (using std::distance): " << dist << std::endl;

        // Example using std::rbegin
        auto it_rbegin = std::rbegin(numbers);
        std::cout << "First element in reverse (using std::rbegin): " << *it_rbegin << std::endl;

        // Example using std::rend
        auto it_rend = std::rend(numbers);
        std::cout << "Last element in reverse (using std::rend): " << *(--it_rend) << std::endl;

        // Example using reverse iterator with std::advance
        auto it_radvance = std::rbegin(numbers);
        std::advance(it_radvance, 3); // Advance reverse iterator by 3 positions
        std::cout << "Element at reverse position 3 (using std::advance with reverse iterator): " << *it_radvance << std::endl;

        // Example using std::distance with reverse iterators
        auto rdist = std::distance(std::rbegin(numbers), std::rend(numbers)); // Get the number of elements between two reverse iterators
        std::cout << "Distance from rbegin to rend (using std::distance with reverse iterators): " << rdist << std::endl;

        // Chaining operations together
        auto it_chain = std::next(std::prev(numbers.end(), 5), 2);
        std::cout << "Element at position chain (using std::next and std::prev together): " << *it_chain << std::endl;
    }
    // C-style array example
    {
        // Create a C-style array of integers
        int numbers[] = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
        constexpr size_t size = sizeof(numbers) / sizeof(numbers[0]);

        // Example using std::next
        int *it_next = std::next(numbers, 3); // Advance iterator by 3 positions
        std::cout << "Element at position 3 (using std::next): " << *it_next << std::endl;

        // Example using std::prev
        int *it_prev = std::prev(numbers + size, 3); // Move iterator back by 3 positions from end
        std::cout << "Element 3 positions before end (using std::prev): " << *it_prev << std::endl;

        // Example using std::advance
        int *it_advance = numbers;
        std::advance(it_advance, 5); // Advance iterator by 5 positions
        std::cout << "Element at position 5 (using std::advance): " << *it_advance << std::endl;

        // Example using std::distance
        int *it_start = numbers;
        int *it_end = numbers + size;
        auto dist = std::distance(it_start, it_end); // Get the number of elements between two iterators
        std::cout << "Distance from begin to end (using std::distance): " << dist << std::endl;

        // Example using std::rbegin
        std::reverse_iterator<int *> it_rbegin(numbers + size);
        std::cout << "First element in reverse (using std::rbegin): " << *it_rbegin << std::endl;

        // Example using std::rend
        std::reverse_iterator<int *> it_rend(numbers);
        std::cout << "Last element in reverse (using std::rend): " << *(--it_rend) << std::endl;

        // Example using reverse iterator with std::advance
        auto it_radvance = std::rbegin(numbers);
        std::advance(it_radvance, 3); // Advance reverse iterator by 3 positions
        std::cout << "Element at reverse position 3 (using std::advance with reverse iterator): " << *it_radvance << std::endl;

        // Example using std::distance with reverse iterators
        auto rdist = std::distance(std::rbegin(numbers), std::rend(numbers)); // Get the number of elements between two reverse iterators
        std::cout << "Distance from rbegin to rend (using std::distance with reverse iterators): " << rdist << std::endl;

        // Chaining operations together
        auto it_chain = std::next(std::prev(numbers + size, 5), 2);
        std::cout << "Element at position chain (using std::next and std::prev together): " << *it_chain << std::endl;
    }
    return 0;
}
