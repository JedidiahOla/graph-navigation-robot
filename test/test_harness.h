// test_harness.h - small test harness, so the tests have no dependencies.
//
// These tests need to run on any machine with a C++ compiler and no robot
// attached, so it is easier to keep them dependency free than to pull in a
// framework that has to be downloaded first.

#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <cstdio>
#include <string>
#include <vector>

struct TestCase {
  const char *name;
  void (*fn)();
};

std::vector<TestCase> &testRegistry();
void testFail(const char *file, int line, const std::string &message);

struct TestRegistrar {
  TestRegistrar(const char *name, void (*fn)()) {
    TestCase testCase = {name, fn};
    testRegistry().push_back(testCase);
  }
};

#define TEST(name)                                    \
  static void name();                                 \
  static TestRegistrar registrar_##name(#name, name); \
  static void name()

#define CHECK(condition)                                     \
  do {                                                       \
    if (!(condition)) {                                      \
      testFail(__FILE__, __LINE__, "expected: " #condition); \
    }                                                        \
  } while (0)

#define CHECK_EQ(actual, expected)                                             \
  do {                                                                         \
    const long long a_ = (long long)(actual);                                  \
    const long long e_ = (long long)(expected);                                \
    if (a_ != e_) {                                                            \
      char buf_[256];                                                          \
      std::snprintf(buf_, sizeof(buf_), "%s == %s  (got %lld, expected %lld)", \
                    #actual, #expected, a_, e_);                               \
      testFail(__FILE__, __LINE__, buf_);                                      \
    }                                                                          \
  } while (0)

#endif  // TEST_HARNESS_H
