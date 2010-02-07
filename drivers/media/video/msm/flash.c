#include <linux/kernel.h>
#include <linux/errno.h>
#include <mach/pmic.h>
#include <mach/camera.h>

int32_t flash_set_led_state(enum msm_camera_led_state_t led_state)
{
	int32_t rc;

  CDBG("flash_set_led_state: %d\n", led_state);
  switch (led_state) {
  case MSM_LED_OFF:
    rc = flash_led_set_current(0);
    break;

  case MSM_LED_LOW:
    rc = flash_led_set_current(30);
    break;

  case MSM_LED_HIGH:
    rc = flash_led_set_current(100);
    break;

  default:
    rc = -EFAULT;
    break;
  }
  CDBG("flash_set_led_state: return %d\n", rc);

  return rc;
}
