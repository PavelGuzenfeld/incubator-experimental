#ifndef LOG_IMPL_HPP
#define LOG_IMPL_HPP

#include "log.hpp"
#include <iostream>
#include <utility>

// Default implementation that does nothing
<template <typename...>
struct null_logger
{
    auto log(auto &&...) const noexcept -> void {}
};

// Dummy test logger
struct test_logger
{
    template <typename... Args>
    auto log(Args &&...args) const -> void
    {
        // Logging to a test output
    }
};

// Template struct to hold the logger instance
template <typename...>
struct log_config
{
    static null_logger &instance()
    {
        static null_logger logger;
        return logger;
    }
};



#endif // LOG_IMPL_HPP
