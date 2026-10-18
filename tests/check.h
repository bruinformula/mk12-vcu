/*
 * check.h
 *
 * Minimal host-side test macros. No framework, no dependencies.
 * A failed check prints file:line and the test keeps going; main() returns
 * nonzero if anything failed, so `make -C tests` fails too.
 */

#ifndef TESTS_CHECK_H_
#define TESTS_CHECK_H_

#include <math.h>
#include <stdio.h>

static int checks_failed = 0;

#define CHECK(cond) do { \
	if (!(cond)) { \
		printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
		checks_failed++; \
	} \
} while (0)

#define CHECK_NEAR(actual, expected, tol) do { \
	double a_ = (actual), e_ = (expected); \
	if (fabs(a_ - e_) > (tol)) { \
		printf("  FAIL %s:%d: %s = %.4f, expected %.4f (+/- %g)\n", \
				__FILE__, __LINE__, #actual, a_, e_, (double)(tol)); \
		checks_failed++; \
	} \
} while (0)

#define RUN(test) do { \
	int before_ = checks_failed; \
	test(); \
	printf("%s %s\n", checks_failed == before_ ? "ok  " : "FAIL", #test); \
} while (0)

#define CHECK_SUMMARY() ( \
	printf("\n%s\n", checks_failed ? "TESTS FAILED" : "all tests passed"), \
	checks_failed ? 1 : 0)

#endif /* TESTS_CHECK_H_ */
