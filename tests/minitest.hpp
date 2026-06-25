#pragma once
// A tiny zero-dependency test harness. Real projects use Catch2 / GoogleTest;
// this keeps the repo buildable anywhere with just g++ and no package manager.
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <cmath>

namespace minitest {

struct Case { std::string name; std::function<void()> fn; };

inline std::vector<Case>& registry() {
    static std::vector<Case> r;
    return r;
}

inline int& failures() { static int f = 0; return f; }

struct Registrar {
    Registrar(const std::string& name, std::function<void()> fn) {
        registry().push_back({name, std::move(fn)});
    }
};

inline int run_all() {
    int passed = 0;
    for (auto& c : registry()) {
        int before = failures();
        try {
            c.fn();
        } catch (const std::exception& e) {
            std::cerr << "  [EXCEPTION] " << c.name << ": " << e.what() << "\n";
            ++failures();
        }
        if (failures() == before) {
            std::cout << "  [PASS] " << c.name << "\n";
            ++passed;
        } else {
            std::cout << "  [FAIL] " << c.name << "\n";
        }
    }
    std::cout << "\n" << passed << "/" << registry().size()
              << " tests passed.\n";
    return failures() == 0 ? 0 : 1;
}

}  // namespace minitest

#define TEST(name) \
    static void name(); \
    static minitest::Registrar reg_##name(#name, name); \
    static void name()

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::cerr << "    CHECK failed: " #cond " (" << __FILE__ << ":" \
                      << __LINE__ << ")\n";                                \
            ++minitest::failures();                                        \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                              \
    do {                                                                   \
        if (std::fabs((a) - (b)) > (tol)) {                               \
            std::cerr << "    CHECK_NEAR failed: " #a " ~= " #b " (got "    \
                      << (a) << " vs " << (b) << ") (" << __FILE__ << ":"   \
                      << __LINE__ << ")\n";                                \
            ++minitest::failures();                                        \
        }                                                                  \
    } while (0)
