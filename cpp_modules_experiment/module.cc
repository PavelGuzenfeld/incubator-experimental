export module simple;
import <string_view>;
import <memory>;
using std::unique_ptr; // not exported
int *parse(std::string_view s)
{ /*…*/
return nullptr;
} // cannot collide with other modules
export namespace simple
{
    auto get_ints(const char *text)
    {
        return unique_ptr<int[]>(parse(text));
    }
}