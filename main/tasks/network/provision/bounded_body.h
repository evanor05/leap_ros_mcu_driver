#pragma once
#include <cstddef>

// Do not dispatch a safety action from a partial HTTP form. In particular,
// action=arm must not execute before its trailing source field has arrived.
template <typename Receive>
bool ReadBoundedBody(char* body, std::size_t capacity, std::size_t length, Receive receive) {
    if (length == 0 || length >= capacity) return false;
    std::size_t total = 0;
    while (total < length) {
        const int count = receive(body + total, length - total);
        if (count <= 0 || static_cast<std::size_t>(count) > length - total) return false;
        total += static_cast<std::size_t>(count);
    }
    body[length] = '\0';
    return true;
}
