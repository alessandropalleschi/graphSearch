#pragma once
// Minimal self-registering test harness (no external dependencies).
#include <cstdio>
#include <exception>
#include <functional>
#include <string>
#include <vector>

struct TestCase {
    const char* name;
    std::function<void()> fn;
};
inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> r;
    return r;
}
struct Registrar {
    Registrar(const char* n, std::function<void()> f) { registry().push_back({n, std::move(f)}); }
};
struct CheckFailure : std::exception {};

#define TEST(name)                                          \
    static void name();                                     \
    static Registrar registrar_##name(#name, name);         \
    static void name()

#define CHECK(c)                                                                          \
    do {                                                                                  \
        if (!(c)) {                                                                       \
            std::fprintf(stderr, "  %s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #c);  \
            throw CheckFailure{};                                                         \
        }                                                                                 \
    } while (0)

#define CHECK_THROWS(expr, Ex)                \
    do {                                      \
        bool threw_ = false;                  \
        try { (void)(expr); } catch (const Ex&) { threw_ = true; } \
        CHECK(threw_);                        \
    } while (0)

inline int runAll() {
    int failed = 0;
    for (auto& t : registry()) {
        try {
            t.fn();
            std::printf("[ OK ] %s\n", t.name);
        } catch (const std::exception& e) {
            if (!dynamic_cast<const CheckFailure*>(&e)) std::fprintf(stderr, "  exception: %s\n", e.what());
            std::printf("[FAIL] %s\n", t.name);
            ++failed;
        }
    }
    std::printf("%zu tests, %d failed\n", registry().size(), failed);
    return failed ? 1 : 0;
}
