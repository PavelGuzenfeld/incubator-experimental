// General Trie (Prefix Tree) Implementation
// Compiler Explorer: Use C++17, add flag: -lfmt
// Common uses: Autocomplete, spell checking, IP routing, dictionary

#include <fmt/core.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

using CharIndex = uint8_t; // Index for character (0-25 for 'a'-'z')

class TrieNode
{
public:
    static constexpr size_t ALPHABET_SIZE = 26; // 'a' to 'z'

    std::unique_ptr<TrieNode> children[ALPHABET_SIZE];
    bool is_end_of_word;      // True if this node marks end of a valid word
    std::optional<int> value; // Optional: store value associated with word

    TrieNode() : is_end_of_word(false)
    {
        for (size_t i = 0; i < ALPHABET_SIZE; i++)
        {
            children[i] = nullptr;
        }
    }
};

class Trie
{
private:
    std::unique_ptr<TrieNode> root;

    // Convert character to index (0-25)
    static CharIndex charToIndex(char ch)
    {
        return static_cast<CharIndex>(ch - 'a');
    }

    // Helper for collecting all words with given prefix
    void collectWords(const TrieNode *node, const std::string &prefix,
                      std::vector<std::string> &results) const
    {
        if (!node)
            return;

        if (node->is_end_of_word)
        {
            results.push_back(prefix);
        }

        for (CharIndex i = 0; i < TrieNode::ALPHABET_SIZE; i++)
        {
            if (node->children[i])
            {
                char ch = static_cast<char>('a' + i);
                collectWords(node->children[i].get(), prefix + ch, results);
            }
        }
    }

public:
    Trie() : root(std::make_unique<TrieNode>()) {}

    // Insert word into trie - O(m) where m = word length
    void insert(const std::string &word, int value = 0)
    {
        TrieNode *current = root.get();

        fmt::print("Inserting: '{}'\n", word);

        for (char ch : word)
        {
            CharIndex index = charToIndex(ch);

            if (!current->children[index])
            {
                current->children[index] = std::make_unique<TrieNode>();
            }
            current = current->children[index].get();
        }

        current->is_end_of_word = true;
        current->value = value;
    }

    // Search for exact word - O(m)
    bool search(const std::string &word) const
    {
        const TrieNode *current = root.get();

        for (char ch : word)
        {
            CharIndex index = charToIndex(ch);
            if (!current->children[index])
            {
                return false;
            }
            current = current->children[index].get();
        }

        return current->is_end_of_word;
    }

    // Check if any word starts with prefix - O(m)
    bool startsWith(const std::string &prefix) const
    {
        const TrieNode *current = root.get();

        for (char ch : prefix)
        {
            CharIndex index = charToIndex(ch);
            if (!current->children[index])
            {
                return false;
            }
            current = current->children[index].get();
        }

        return true; // Found prefix path
    }

    // Get value associated with word - O(m)
    std::optional<int> getValue(const std::string &word) const
    {
        const TrieNode *current = root.get();

        for (char ch : word)
        {
            CharIndex index = charToIndex(ch);
            if (!current->children[index])
            {
                return std::nullopt;
            }
            current = current->children[index].get();
        }

        if (current->is_end_of_word)
        {
            return current->value;
        }
        return std::nullopt;
    }

    // Autocomplete: Find all words with given prefix - O(p + n)
    // where p = prefix length, n = number of nodes in subtree
    std::vector<std::string> autocomplete(const std::string &prefix) const
    {
        std::vector<std::string> results;
        const TrieNode *current = root.get();

        // Navigate to prefix node
        for (char ch : prefix)
        {
            CharIndex index = charToIndex(ch);
            if (!current->children[index])
            {
                return results; // Empty - prefix doesn't exist
            }
            current = current->children[index].get();
        }

        // Collect all words from this point
        collectWords(current, prefix, results);
        return results;
    }

    // Delete word - O(m)
    bool remove(const std::string &word)
    {
        return removeHelper(root.get(), word, 0);
    }

private:
    bool removeHelper(TrieNode *node, const std::string &word, size_t depth)
    {
        if (!node)
            return false;

        // Reached end of word
        if (depth == word.size())
        {
            if (!node->is_end_of_word)
            {
                return false; // Word doesn't exist
            }

            node->is_end_of_word = false;
            node->value.reset();

            // Check if node has no children (can be deleted)
            for (size_t i = 0; i < TrieNode::ALPHABET_SIZE; i++)
            {
                if (node->children[i])
                    return false;
            }
            return true; // Can delete this node
        }

        CharIndex index = charToIndex(word[depth]);
        if (!node->children[index])
        {
            return false; // Path doesn't exist
        }

        bool shouldDeleteChild = removeHelper(node->children[index].get(), word, depth + 1);

        if (shouldDeleteChild)
        {
            node->children[index].reset();

            // Check if current node can also be deleted
            if (!node->is_end_of_word)
            {
                for (size_t i = 0; i < TrieNode::ALPHABET_SIZE; i++)
                {
                    if (node->children[i])
                        return false;
                }
                return true;
            }
        }

        return false;
    }
};

int main()
{
    Trie trie;

    fmt::print("=== GENERAL TRIE EXAMPLE ===\n\n");

    // 1. INSERT WORDS
    fmt::print("1. INSERTING WORDS\n");
    fmt::print("------------------\n");
    trie.insert("cat", 100);
    trie.insert("car", 200);
    trie.insert("card", 300);
    trie.insert("care", 400);
    trie.insert("careful", 500);
    trie.insert("dog", 600);
    trie.insert("dodge", 700);
    fmt::print("\n");

    // 2. SEARCH
    fmt::print("2. SEARCHING FOR WORDS\n");
    fmt::print("----------------------\n");
    fmt::print("Search 'car': {}\n", trie.search("car") ? "FOUND" : "NOT FOUND");
    fmt::print("Search 'care': {}\n", trie.search("care") ? "FOUND" : "NOT FOUND");
    fmt::print("Search 'card': {}\n", trie.search("card") ? "FOUND" : "NOT FOUND");
    fmt::print("Search 'cards': {}\n", trie.search("cards") ? "FOUND" : "NOT FOUND");
    fmt::print("\n");

    // 3. PREFIX SEARCH
    fmt::print("3. PREFIX SEARCH\n");
    fmt::print("----------------\n");
    fmt::print("Starts with 'car': {}\n", trie.startsWith("car") ? "YES" : "NO");
    fmt::print("Starts with 'care': {}\n", trie.startsWith("care") ? "YES" : "NO");
    fmt::print("Starts with 'cat': {}\n", trie.startsWith("cat") ? "YES" : "NO");
    fmt::print("Starts with 'dog': {}\n", trie.startsWith("dog") ? "YES" : "NO");
    fmt::print("Starts with 'bat': {}\n", trie.startsWith("bat") ? "YES" : "NO");
    fmt::print("\n");

    // 4. GET VALUES
    fmt::print("4. GET ASSOCIATED VALUES\n");
    fmt::print("------------------------\n");
    auto val1 = trie.getValue("car");
    fmt::print("Value of 'car': {}\n", val1 ? *val1 : -1);
    auto val2 = trie.getValue("careful");
    fmt::print("Value of 'careful': {}\n", val2 ? *val2 : -1);
    fmt::print("\n");

    // 5. AUTOCOMPLETE
    fmt::print("5. AUTOCOMPLETE\n");
    fmt::print("---------------\n");
    auto suggestions1 = trie.autocomplete("car");
    fmt::print("Autocomplete 'car': ");
    for (const auto &word : suggestions1)
    {
        fmt::print("{} ", word);
    }
    fmt::print("\n");

    auto suggestions2 = trie.autocomplete("do");
    fmt::print("Autocomplete 'do': ");
    for (const auto &word : suggestions2)
    {
        fmt::print("{} ", word);
    }
    fmt::print("\n\n");

    // 6. DELETE
    fmt::print("6. DELETE WORD\n");
    fmt::print("--------------\n");
    fmt::print("Deleting 'car'\n");
    trie.remove("car");
    fmt::print("Search 'car' after deletion: {}\n", trie.search("car") ? "FOUND" : "NOT FOUND");
    fmt::print("Search 'card' after deletion: {}\n", trie.search("card") ? "FOUND" : "NOT FOUND");
    fmt::print("Starts with 'car' after deletion: {}\n", trie.startsWith("car") ? "YES" : "NO");

    return 0;
}