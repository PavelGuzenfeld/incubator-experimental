# Advanced C++ Continuation Monad

This project demonstrates an advanced **Continuation Monad** in C++23 for managing complex asynchronous operations. It provides a clean, chainable API for building concurrent data pipelines with robust, expressive error handling.

This implementation allows you to:

  * Run multiple asynchronous pipelines concurrently.
  * Chain operations without blocking or nested callbacks.
  * Handle errors gracefully using a "railway-oriented" approach.
  * Implement complex recovery logic with a rich error-handling API.

-----

## The API and Mechanism

The `Continuation` class wraps a `std::future<std::expected<T, Error>>`, encapsulating a value that will be computed asynchronously and may result in either a success (`T`) or an `Error`.

### Core Chaining Methods

The API provides three main methods for building pipelines:

#### `and_then(Func f)`

This is for the **success path**.

  * **What it does:** Chains a function that will only be executed if the previous step in the pipeline succeeded.
  * **Input:** A function that takes the success value (`T`) and returns a new `std::expected<U, Error>`.
  * **Behavior:** If the pipeline is in an error state, `and_then` is skipped entirely.

-----

#### `or_else(Func f)`

This is for the **failure path**, enabling complex recovery.

  * **What it does:** Chains a function that will only be executed if the previous step failed.
  * **Input:** A function that takes an `Error` object and can attempt to recover. It must return a `std::expected<T, Error>`.
  * **Behavior:** If the recovery function returns a success value, the pipeline is switched back to the success track. If it returns another error, that new error is propagated.

-----

#### `or_value(T default_value)`

This is a simplified recovery mechanism for the **failure path**.

  * **What it does:** Replaces any error with a provided default success value.
  * **Input:** A default value of type `T`.
  * **Behavior:** This method always switches a failing pipeline back to the success track. It's a convenient shortcut for a simple `or_else` recovery.

-----

## How to Compile and Run

### Dependencies

  * A **C++23 compatible compiler** (e.g., GCC 12+, Clang 15+).
  * The **{fmt} library**.
  * On Linux/macOS, you may need to link the **pthread** library.

### Compilation Command

1.  **Get {fmt} library (if needed):**
    ```bash
    git clone https://github.com/fmtlib/fmt.git
    ```
2.  **Compile the source:**
    ```bash
    # Replace /path/to/fmt/include with the actual path
    g++ -std=c++23 -I/path/to/fmt/include main.cpp -o async_pipelines -lpthread
    ```
3.  **Run the executable:**
    ```bash
    ./async_pipelines
    ```

### Easiest Method: Godbolt 🚀

The simplest way to compile and run this example is to use the pre-configured Godbolt link, which has all dependencies ready.

[**Open in Godbolt**](https://www.google.com/search?q=https://godbolt.org/z/s8qcrjWPK)