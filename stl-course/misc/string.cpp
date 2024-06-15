#include <fmt/core.h>
#include <iostream>
#include <string>

int main()
{
    // string declaration:

    std::string name = "James Slocum"; // Assingment construcor
    std::string n2("James Slucum");
    std::string n4{'J', 'a', 'm', 'e', 's'}; // initializer list constractor
    std::string last(name, 6, 6);            // offset and number of char Slocum
    std::string line(20, '-');               // repeat '-' 20 times

    // string functions:
    std::string revens(name.rbegin(), name.rend());

    auto const_char_ptr = "const char ptr"; // const char ptr

    fmt::print("{}\n", const_char_ptr);
    using namespace std;
    auto str = "string"s; // for using operator""s using std::string is required
    fmt::print("{}\n", str);

    cout << "Comma at: " << ("Hello, How are you?"s).find(",") << '\n';

    cout << line << '\n';

    // Append to string
    string about = "is a programmer";
    name.push_back(' ');
    for (char c : about)
    {
        name.push_back(c);
    }

    cout << name << '\n';

    // Insert into strings
    name.insert(0, "I have heard that ");
    cout << name << '\n';

    // Finding strings
    std::size_t pos = name.find(n4);
    if (pos != std::string::npos)
    {
        cout << "Found first name at: " << pos << '\n';
    }

    // Replace some string
    pos = name.find("programmer");
    name.replace(pos, ("programmer"s).length(), "snowboarder");

    pos = name.find("heard");
    name.erase(pos, ("heard"s).length());
    name.insert(pos, "read somewhere");
    cout << name << '\n';
    return 0;
}