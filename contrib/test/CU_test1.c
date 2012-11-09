//#include "stdio.h"
#include <CUnit/Basic.h>
#include "../../services/heating_controller/heating_ctrl.h"
//#include <avr/pgmspace.h>


/* Stubs
 *
 */
//int
//printf_P (const char *fmt, ...)
//{
//  char *f = printffmtfix(fmt);
//va_list va;
//va_start (va, fmt);
//int r = vprintf (f, va);
//va_end (va);
//
//g_free (f);
//return r;
//}

/* End stubs
 *
 */

/* The suite initialization function.
 * Opens the temporary file used by the tests.
 * Returns zero on success, non-zero otherwise.
 */
int init_suite1(void)
{
   /* Here we put code that will run before
    * the entire test suit runs. */
  return 0;
}


/* The suite cleanup function.
 * Closes the temporary file used by the tests.
 * Returns zero on success, non-zero otherwise.
 */
int clean_suite1(void)
{
  /* Here we put code that will run after
   * the entire test suit runs. */
  return 0;
}


/****** Test functions ********/

void test_heating_ctrl_init(void){
 // heating_ctrl_init();

  CU_ASSERT(1 == 0);
}


/******  End Test functions ********/


int main(){
  printf("Hello, testworld!");
    CU_pSuite pSuite = NULL;

    /* initialize the CUnit test registry */
    if (CUE_SUCCESS != CU_initialize_registry())
       return CU_get_error();

    /* add a suite to the registry */
    pSuite = CU_add_suite("Suite_1", init_suite1, clean_suite1);
    if (NULL == pSuite) {
       CU_cleanup_registry();
       return CU_get_error();
    }

    /* add the tests to the suite */
      if ((NULL == CU_add_test(pSuite, "test of heating_ctrl_init()", test_heating_ctrl_init))
  //     (NULL == CU_add_test(pSuite, "test of getTwo()", testGetTwo)) ||
    //   (NULL == CU_add_test(pSuite, "test of getThreeString()", testGetThreeString)) ||
  //     (NULL == CU_add_test(pSuite, "test of getFourString()", testGetFourString)))
          )
      {
       CU_cleanup_registry();
       return CU_get_error();
      }

      /* Run all tests using the CUnit Basic interface */
      CU_basic_set_mode(CU_BRM_VERBOSE);
      CU_basic_run_tests();
      CU_cleanup_registry();
      return CU_get_error();
  }

