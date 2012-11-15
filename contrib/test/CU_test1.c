//#include "stdio.h"
#include <CUnit/Basic.h>
#include "CU_test1.h"
#include "services/heating_controller/heating_ctrl.h"
#include "core/eeprom.h"
#include "hardware/onewire/onewire.h"
#include "hardware/pwm/pwm.h"
//#include <avr/pgmspace.h>

extern sensor_data_t sensors[N_SENSORS];
extern heating_ctrl_params_t heating_ctrl_params_ram;


void eeprom_read_block(){}
int8_t ow_temp_read_scratchpad(ow_rom_code_t * rom, ow_temp_scratchpad_t * scratchpad){
  return 0;
}
int16_t  ow_temp_normalize(ow_rom_code_t * rom, ow_temp_scratchpad_t * sp){
  return 0;
}
int8_t ow_temp_start_convert(ow_rom_code_t * rom, uint8_t wait){
  return 0;
}
void eeprom_write_block_hack (void *dst, const void *src, size_t n)
{}

uint8_t eeprom_get_chksum (void){
  return 0;
}

void setpwm(char channel, uint8_t setval){

}

int sscanf_P(const char *__buf, const char *__fmt, ...){
  return 0;
}


/* Stubs
 *
 */

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
  heating_ctrl_init();

  CU_ASSERT(sensors[SENSOR_T_ROOM].rom.raw == 0x5100000271fadb28);
}

void test_pid_controller(void){
  long cnt;

  /* Prerequisites */
  sensors[SENSOR_T_ROOM].signal = 20<<4;
  heating_ctrl_params_ram.pid_room.I = 1;
  heating_ctrl_params_ram.pid_room.Igain = 1;
  heating_ctrl_params_ram.pid_room.Pgain = 1;
  heating_ctrl_params_ram.pid_room.u = 0;
  heating_ctrl_params_ram.pid_room.uMax = 40<<4;
  heating_ctrl_params_ram.pid_room.uMin = 0;
  heating_ctrl_params_ram.t_target_room = 20<<4;

  /* Test 1:
   * The controller output shall stay at 0 when target temperature
   * is reached
   */
  for(cnt=0;cnt<10;cnt++){
    pid_controller(&heating_ctrl_params_ram.pid_room,
        heating_ctrl_params_ram.t_target_room, &sensors[SENSOR_T_ROOM]);


    CU_ASSERT(heating_ctrl_params_ram.pid_room.u == 0);
  }
  /* Test 2:
   * The controller output shall be limited at uMax
   */
  sensors[SENSOR_T_ROOM].signal = 19;

  for(cnt=0;cnt<10;cnt++){
    pid_controller(&heating_ctrl_params_ram.pid_room,
        heating_ctrl_params_ram.t_target_room, &sensors[SENSOR_T_ROOM]);

    printf("Output: %d\n",heating_ctrl_params_ram.pid_room.u);
    CU_ASSERT(heating_ctrl_params_ram.pid_room.u <= heating_ctrl_params_ram.pid_room.uMax);
  }
}


/******  End Test functions ********/


int main(){
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
      if ((NULL == CU_add_test(pSuite, "test of heating_ctrl_init()", test_heating_ctrl_init)) ||
       (NULL == CU_add_test(pSuite, "test of pid_controller()", test_pid_controller))
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

