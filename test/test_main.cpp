#include <cstdio>

#include "test_harness.h"

namespace {
int g_failures = 0;
}

std::vector<TestCase> &testRegistry() {
  static std::vector<TestCase> registry;
  return registry;
}

void testFail(const char *file, int line, const std::string &message) {
  std::printf("    FAIL %s:%d  %s\n", file, line, message.c_str());
  ++g_failures;
}

int main() {
  std::vector<TestCase> &tests = testRegistry();
  int failedCases = 0;

  for (size_t i = 0; i < tests.size(); ++i) {
    const int before = g_failures;
    std::printf("  %s\n", tests[i].name);
    tests[i].fn();
    if (g_failures > before) {
      ++failedCases;
    }
  }

  std::printf("\n%zu tests run, %d failed, %d failed assertions\n",
              tests.size(), failedCases, g_failures);
  return (g_failures == 0) ? 0 : 1;
}
