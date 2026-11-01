/*
 * minitest.h - a tiny, dependency-free unit-test harness (C++11).
 * Usage:  TEST(name) { CHECK(cond); CHECK_EQ(a, b); }   and  RUN_ALL_TESTS().
 */
#ifndef KILAT_MINITEST_H
#define KILAT_MINITEST_H

#include <stdio.h>
#include <string.h>

namespace minitest {
struct Case { const char *name; void (*fn)(); };
inline Case *registry() { static Case cases[512]; return cases; }
inline int &count()     { static int n = 0; return n; }
inline int &failures()  { static int f = 0; return f; }
inline int &checks()    { static int c = 0; return c; }
inline const char *&current() { static const char *n = ""; return n; }
struct Register {
  Register(const char *name, void (*fn)()) {
    registry()[count()++] = Case{name, fn};
  }
};
}  /* namespace minitest */

#define TEST(name)                                                          \
  static void test_##name();                                                \
  static minitest::Register reg_##name(#name, test_##name);                 \
  static void test_##name()

#define CHECK(cond)                                                         \
  do {                                                                      \
    minitest::checks()++;                                                   \
    if (!(cond)) {                                                          \
      minitest::failures()++;                                               \
      printf("  FAIL [%s] %s:%d: %s\n", minitest::current(), __FILE__,      \
             __LINE__, #cond);                                              \
    }                                                                       \
  } while (0)

#define CHECK_EQ(a, b)                                                      \
  do {                                                                      \
    minitest::checks()++;                                                   \
    long long va_ = (long long)(a), vb_ = (long long)(b);                   \
    if (va_ != vb_) {                                                       \
      minitest::failures()++;                                               \
      printf("  FAIL [%s] %s:%d: %s == %s  (got %lld, expected %lld)\n",    \
             minitest::current(), __FILE__, __LINE__, #a, #b, va_, vb_);    \
    }                                                                       \
  } while (0)

#define CHECK_STR(a, b)                                                     \
  do {                                                                      \
    minitest::checks()++;                                                   \
    if (strcmp((a), (b)) != 0) {                                            \
      minitest::failures()++;                                               \
      printf("  FAIL [%s] %s:%d: \"%s\" != \"%s\"\n", minitest::current(),  \
             __FILE__, __LINE__, (a), (b));                                 \
    }                                                                       \
  } while (0)

#define RUN_ALL_TESTS()                                                     \
  ([]() -> int {                                                            \
    for (int i = 0; i < minitest::count(); i++) {                           \
      minitest::current() = minitest::registry()[i].name;                   \
      minitest::registry()[i].fn();                                         \
    }                                                                       \
    printf("%d tests, %d checks, %d failures\n", minitest::count(),         \
           minitest::checks(), minitest::failures());                       \
    return minitest::failures() == 0 ? 0 : 1;                               \
  })()

#endif /* KILAT_MINITEST_H */
