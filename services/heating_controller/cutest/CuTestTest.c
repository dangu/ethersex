#include <assert.h>
#include <setjmp.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "CuTest.h"

/*-------------------------------------------------------------------------*
 * Helper functions
 *-------------------------------------------------------------------------*/

#define CompareAsserts(tc, message, expected, actual)  X_CompareAsserts((tc), __FILE__, __LINE__, (message), (expected), (actual))



/*-------------------------------------------------------------------------*
 * CuTest Test
 *-------------------------------------------------------------------------*/

void TestPasses(CuTest* tc)
{
	CuAssert(tc, "test should pass", 1 == 0 + 1);
}

void zTestFails(CuTest* tc)
{
	CuAssert(tc, "test should fail", 1 == 1 + 1);
}



void TestFail(CuTest* tc)
{
	jmp_buf buf;
	int pointReached = 0;
	CuTest* tc2 = CuTestNew("TestFails", zTestFails);
	tc2->jumpBuf = &buf;
	if (setjmp(buf) == 0)
	{
		CuFail(tc2, "hello world");
		pointReached = 1;
	}
	CuAssert(tc, "point was not reached", pointReached == 0);
}

void Test1(CuTest* tc)
{
  CuAssert(tc, "Something here?", 0 == 0);
}
/*-------------------------------------------------------------------------*
 * main
 *-------------------------------------------------------------------------*/

CuSuite* CuGetSuite(void)
{
	CuSuite* suite = CuSuiteNew();


//	SUITE_ADD_TEST(suite, TestFail);
  SUITE_ADD_TEST(suite, Test1);


	return suite;
}
