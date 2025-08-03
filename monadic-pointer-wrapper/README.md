# C++ Monadic Pointer Wrapper (`CPtr`)

This project provides a C++23 implementation of a monadic pointer wrapper named `CPtr`. It's inspired by the concepts presented in Robert Schimkowitsch's ACCU 2025 talk, "Safe and Readable C++ Code: Monadic Operations in C++23".

The primary goal is to simplify error handling when dealing with sequences of operations that might return null pointers, avoiding deeply nested `if` statements (the "pyramid of doom").

-----

## The Logic and Mechanism

### The Problem: The Pyramid of Doom

When accessing nested data through pointers, each access must be checked for `nullptr` to avoid segmentation faults. This often leads to code that is hard to read and maintain:

```cpp
// Traditional null-checking
if (auto* element = getElementPtr(db, key)) {
    if (auto* table = getTablePtr(*element, tableName)) {
        if (auto* cell = getCellPtr(*table, loc)) {
            if (auto* value = getNumericCellValue(*cell)) {
                // ... finally do something with value
            }
        }
    }
}
```

### The Monadic Solution: Railway-Oriented Programming 🛤️

The `CPtr` monad wraps a raw pointer and encapsulates the null-checking logic. It allows you to chain operations together in a clean, linear sequence. If any step in the chain returns a `nullptr`, the entire chain "short-circuits" and gracefully stops, returning an empty state.

This approach is often called **Railway-Oriented Programming**. You can imagine two tracks:

  * **The "Success" Track**: As long as each function returns a valid pointer, the computation stays on this track.
  * **The "Failure" Track**: The moment a function returns `nullptr`, the computation switches to this track and bypasses all subsequent operations.

### Core Operations

The `CPtr` wrapper provides two main monadic operations to build these chains:

1.  **`and_then(...)`**

      * This is the monadic **bind** operation (`>>=`).
      * It takes a function that accepts the wrapped value and returns a **new raw pointer**.
      * If the `CPtr` contains a valid pointer, `and_then` calls the function and wraps the resulting pointer in a new `CPtr` to continue the chain.
      * If the `CPtr` is empty (contains `nullptr`), it immediately returns an empty `CPtr`, effectively short-circuiting the chain.

2.  **`transform(...)`**

      * This is the monadic **map** operation (or `fmap`).
      * It's used at the **end of a chain** to convert the final valid pointer into a different value.
      * It takes a function that accepts the wrapped value and returns a **plain value** (not a pointer).
      * The result is automatically wrapped in a `std::optional`. If the chain had already failed, it returns `std::nullopt`.

-----

## How to Compile and Run

### Dependencies

  * A **C++23 compatible compiler** (e.g., GCC 12+, Clang 15+).
  * The **{fmt} library** for formatted output.

### Compilation Command

If you have the `{fmt}` library installed, you can compile the `main.cpp` file from the command line.

1.  **Clone {fmt} (if you don't have it):**

    ```bash
    git clone https://github.com/fmtlib/fmt.git
    ```

2.  **Compile the source:**

    ```bash
    # Replace /path/to/fmt/include with the actual path to the fmt include directory
    g++ -std=c++23 -I/path/to/fmt/include main.cpp -o monadic_example
    ```

3.  **Run the executable:**

    ```bash
    ./monadic_example
    ```

### Easiest Method: Godbolt 🚀

The simplest way to compile and run this example is to use the pre-configured Godbolt link. It includes the C++23 compiler and the `{fmt}` library.

[**Open in Godbolt**](https://www.google.com/search?q=https://godbolt.org/z/KExr4bE43)