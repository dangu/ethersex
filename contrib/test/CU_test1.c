//#include "stdio.h"
#include <CUnit/Basic.h>
#include "CU_test1.h"
#include "services/heating_controller/heating_ctrl.h"
#include "core/eeprom.h"
#include "hardware/onewire/onewire.h"
#include "hardware/pwm/pwm.h"
#include "core/bit-macros.h"
//#include <avr/pgmspace.h>

extern sensor_data_t sensors[N_SENSORS];
extern heating_ctrl_params_t heating_ctrl_params_ram;


void eeprom_read_block(){}
int8_t ow_temp_read_scratchpad(ow_rom_code_t * rom, ow_temp_scratchpad_t * scratchpad){
  return 0;
}
/* Stub
ow_temp_t  ow_temp_normalize(ow_rom_code_t * rom, ow_temp_scratchpad_t * sp){
  ow_temp_t ret = { 0, 0 };
  return ret;
}
*/

ow_temp_t
ow_temp_normalize(ow_rom_code_t * rom, ow_temp_scratchpad_t * sp)
{
  ow_temp_t ret = { 0, 0 };
  if (rom->family == OW_FAMILY_DS1820)
  {
      /* This calculation is done in deg*256 */
    uint16_t temp = (int16_t) ((sp->temperature & 0xfffe) << 7) - 0x40 +
      (((sp->count_per_c - sp->count_remain) << 8) / sp->count_per_c);
    /* At this point, temp has the format
     *  S  d6 d5 d4 | d3 d2  d1  d0 | d-1 d-2 0  0 | 0 0 0 0
     */
    ret.val = ((int8_t) HI8(temp)) * 10 + HI8(((temp & 0x00ff) * 10) + 0x80);
    /* implicitly: ret.twodigits = 0; */
  }
  else if (rom->family == OW_FAMILY_DS1822 ||
           rom->family == OW_FAMILY_DS18B20)
  {
    uint16_t temp = (int16_t) (sp->temperature << 4);
    ret.val =
      ((int8_t) HI8(temp)) * 100 + HI8(((temp & 0x00ff) * 100) + 0x80);
    ret.twodigits = 1;
  }
  return ret;
}

/* Old implementation of the normalize function.
 * It probably returns 16th of degrees
 */
int16_t
ow_temp_normalize_old(ow_rom_code_t * rom, ow_temp_scratchpad_t * sp)
{
  if (rom->family == OW_FAMILY_DS1820)
    return (int16_t) ((sp->temperature & 0xfffe) << 7) - 0x40 +
      (((sp->count_per_c - sp->count_remain) << 8) / sp->count_per_c);
  else if (rom->family == OW_FAMILY_DS1822 ||
           rom->family == OW_FAMILY_DS18B20)
    return (int16_t) (sp->temperature << 4);
  else
    return -1;
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

/* Compare temperature
 *
 */

void
compare_temp(ow_temp_scratchpad_t* spPtr, ow_rom_code_t* romPtr, int tTarget,
    uint8_t hi, uint8_t lo, uint8_t remain)
{
  /* Fake a scratchpad... */
  spPtr->temperature_high = hi;
  spPtr->temperature_low =  lo;
  spPtr->count_per_c = 16;
  spPtr->count_remain = remain;
  printf("Scratchpad raw:: 0x%02X %02X\n", spPtr->temperature_high,
      spPtr->temperature_low);

  /* Commented out the new implementation
  ow_temp_t temp = ow_temp_normalize(romPtr, spPtr);
  printf("Ttarget: %d Tcalc %d\n", tTarget, temp.val);
  CU_ASSERT(temp.val == tTarget);
*/
  int16_t temp = ow_temp_normalize_old(romPtr, spPtr);
  if(romPtr->family == OW_FAMILY_DS18B20)
    {
    temp = (temp/16*100)/16;
    }
  else
  {
    temp = (temp/16*10)/16;
  }
  printf("Ttarget: %d Tcalc %d\n", tTarget, temp);
  CU_ASSERT(temp == tTarget);
}

/* Test onewire functions
 *
 * Use different onewire sensor types to test the
 * conversion strategy.
 *
 */

void test_onewire(void){
  ow_temp_scratchpad_t sp;
  sensor_data_t sensor;
  ow_rom_code_t *romPtr;

  /* Fake a sensor type... */
  romPtr = &sensor.rom;
  romPtr->family = OW_FAMILY_DS1820;

  /* Compare temperature
   *
   * Test data (from DS18s20 datasheet)
   * +85.0* 0000 0000 1010 1010 00AAh
   * +25.0 0000 0000 0011 0010 0032h
   * +0.5 0000 0000 0000 0001 0001h
   * 0 0000 0000 0000 0000 0000h
   * -0.5 1111 1111 1111 1111 FFFFh
   * -25.0 1111 1111 1100 1110 FFCEh
   * -55.0 1111 1111 1001 0010 FF92h
   */
  compare_temp(&sp, romPtr, 5, 0x00, 0x01, 0x4);
  compare_temp(&sp, romPtr, 850, 0x00, 0xAA, 0xC);
  compare_temp(&sp, romPtr, 250, 0x00, 0x32, 0xC);
  compare_temp(&sp, romPtr, 5, 0x00, 0x01, 0x4);
  compare_temp(&sp, romPtr, 0, 0x00, 0x00, 0xC);
  compare_temp(&sp, romPtr, -5, 0xFF, 0xFF, 0x4);
  compare_temp(&sp, romPtr, -250, 0xFF, 0xCE, 0xC);
  compare_temp(&sp, romPtr, -550, 0xFF, 0x92, 0xC);

  /* DS18B20
   *
   * +125 0000 0111 1101 0000 07D0h
   * +85* 0000 0101 0101 0000 0550h
   * +25.0625 0000 0001 1001 0001 0191h
   * +10.125 0000 0000 1010 0010 00A2h
   * +0.5 0000 0000 0000 1000 0008h
   * 0 0000 0000 0000 0000 0000h
   * -0.5 1111 1111 1111 1000 FFF8h
   * -10.125 1111 1111 0101 1110 FF5Eh
   * -25.0625 1111 1110 0110 1111 FE6Fh
   * -55 1111 1100 1001 0000 FC90h
   */
  romPtr->family = OW_FAMILY_DS18B20;
  compare_temp(&sp, romPtr, 12500, 0x07, 0xD0, 0x0);
  compare_temp(&sp, romPtr, 8500, 0x05, 0x50, 0x0);
  compare_temp(&sp, romPtr, 2506, 0x01, 0x91, 0x0);
  compare_temp(&sp, romPtr, 1013, 0x00, 0xA2, 0x0);
  compare_temp(&sp, romPtr, 50  , 0x00, 0x08, 0x0);
  compare_temp(&sp, romPtr, 0   , 0x00, 0x00, 0x0);
  compare_temp(&sp, romPtr, -50 , 0xFF, 0xF8, 0x0);
  compare_temp(&sp, romPtr, -1013, 0xFF, 0x5E, 0x0); /* This fails. It rounds to -1012. Is that ok? */
  compare_temp(&sp, romPtr, -2506, 0xFE, 0x6F, 0x0);
  compare_temp(&sp, romPtr, -5500, 0xFC, 0x90, 0x0);
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
       (NULL == CU_add_test(pSuite, "test of pid_controller()", test_pid_controller)) ||
       (NULL == CU_add_test(pSuite, "test of onewire", test_onewire))
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

