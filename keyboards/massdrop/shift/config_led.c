#ifdef RGB_MATRIX_ENABLE
#include "shift.h"

#include "led_matrix.h"
#include "rgb_matrix.h"
#include "config_led.h"

// This table can be almost-automatically derived from ISSI3733_LED_MAP that is
// defined in config_led.h
//
// scan in the following equations refers to the scan variable of ISSI3733_LED_MAP
//   col = (uint8_t)(scan / 8)
//   row = (uint8_t)(scan % 8)
//
// You can calculate the (0-244, 0-64) x/y values from the x/y values defined in
// ISSI3733_LED_MAP with the following formula:
//   uint8_t rgb_x = ((ISSI3733_LED_MAP[i].x - MIN_X) / (MAX_X - MIN_X)) * 224;
//   uint8_t rgb_y = ((ISSI3733_LED_MAP[i].y - MIN_Y) / (MAX_Y - MIN_Y)) * 64; //TODO: 64 - this?
// Where the min/max vars are the minimum and maximum "bounds" of x/y values
// present in ISSI3733_LED_MAP
//
// The row/col values need to be manually tweaked though, compensating for the
// "empty" cells that are a product of larger keys
//

led_config_t g_led_config = {
  {

    {    0,		1,	2,	4,	5,	6,	7,	9,	11,	NO_LED,	14,	15,	NO_LED	},
    {    21,	22,	3,	26,	28,	29,	8,	10,	12,	17,	18,	19,	NO_LED	},
    {    39,	23,	24,	27,	45,	30,	31,	33,	34,	35,	36,	37,	38		},
    {    57,	40,	25,	43,	46,	47,	32,	50,	51,	53,	54,	55,	56		},
    {    74,	41,	42,	44,	63,	48,	49,	67,	52,	70,	71,	72,	73		},
    {    91,	58,	60,	61,	64,	65,	66,	83,	68,	86,	87,	88,	90		},
    {    92,	59,	76,	62,	79,	81,	82,	84,	69,	89,	101,97,	98		},
    {    93,	75,	77,	78,	94,	80,	95,	96,	85,	99,	100,NO_LED,	NO_LED  }
  },
  {
	  {6, 61 },
	  {20, 61 },
	  {31, 61 },
	  {43, 61 },
	  {54, 61 },
	  {69, 61 },
	  {80, 61 },
	  {91, 61 },
	  {103, 61 },
	  {117, 61 },
	  {129, 61 },
	  {140, 61 },
	  {152, 61 },
	  {183, 61 },
	  {206, 61 },
	  {166, 61 },
	  {194, 61 },
	  {217, 61 },
	  {6, 49 },
	  {17, 49 },
	  {29, 49 },
	  {40, 49 },
	  {51, 49 },
	  {63, 49 },
	  {74, 49 },
	  {86, 49 },
	  {97, 49 },
	  {109, 49 },
	  {120, 49 },
	  {132, 49 },
	  {143, 49 },
	  {160, 49 },
	  {183, 49 },
	  {194, 49 },
	  {206, 49 },
	  {217, 49 },
	  {9, 39 },
	  {23, 39 },
	  {34, 39 },
	  {46, 39 },
	  {57, 39 },
	  {69, 39 },
	  {80, 39 },
	  {91, 39 },
	  {103, 39 },
	  {114, 39 },
	  {126, 39 },
	  {137, 39 },
	  {149, 39 },
	  {163, 39 },
	  {183, 39 },
	  {194, 39 },
	  {206, 39 },
	  {217, 39 },
	  {10, 29 },
	  {26, 29 },
	  {37, 29 },
	  {49, 29 },
	  {60, 29 },
	  {71, 29 },
	  {83, 29 },
	  {94, 29 },
	  {106, 29 },
	  {117, 29 },
	  {129, 29 },
	  {140, 29 },
	  {159, 29 },
	  {183, 29 },
	  {194, 29 },
	  {206, 29 },
	  {217, 29 },
	  {13, 20 },
	  {31, 20 },
	  {43, 20 },
	  {54, 20 },
	  {66, 20 },
	  {77, 20 },
	  {89, 20 },
	  {100, 20 },
	  {112, 19 },
	  {123, 20 },
	  {134, 20 },
	  {150, 20 },
	  {183, 20 },
	  {194, 20 },
	  {206, 20 },
	  {169, 17 },
	  {214, 12 },
	  {7, 10 },
	  {21, 10 },
	  {36, 10 },
	  {79, 10 },
	  {123, 10 },
	  {140, 10 },
	  {194, 10 },
	  {206, 10 },
	  {157, 7 },
	  {169, 7 },
	  {180, 7 },
	  {1, 63 },
	  {6, 64 },
	  {26, 64 },
	  {35, 64 },
	  {45, 64 },
	  {54, 64 },
	  {64, 64 },
	  {74, 64 },
	  {83, 64 },
	  {93, 64 },
	  {102, 64 },
	  {112, 64 },
	  {122, 64 },
	  {131, 64 },
	  {141, 64 },
	  {150, 64 },
	  {160, 64 },
	  {170, 64 },
	  {179, 64 },
	  {189, 64 },
	  {198, 64 },
	  {218, 64 },
	  {222, 63 },
	  {224, 58 },
	  {224, 51 },
	  {224, 43 },
	  {224, 35 },
	  {224, 28 },
	  {224, 20 },
	  {224, 12 },
	  {224, 5 },
	  {222, 0 },
	  {218, 0 },
	  {208, 0 },
	  {198, 0 },
	  {189, 0 },
	  {179, 0 },
	  {170, 0 },
	  {160, 0 },
	  {150, 0 },
	  {141, 0 },
	  {131, 0 },
	  {122, 0 },
	  {112, 0 },
	  {102, 0 },
	  {93, 0 },
	  {83, 0 },
	  {74, 0 },
	  {64, 0 },
	  {54, 0 },
	  {45, 0 },
	  {35, 0 },
	  {26, 0 },
	  {16, 0 },
	  {6, 0  },
	  {1, 0  },
	  {0, 5  },
	  {0, 12 },
	  {0, 20 },
	  {0, 28 },
	  {0, 35 },
	  {0, 43 },
	  {0, 51 },
	  {0, 58 },
	  {174, 61 },
	  {174, 58 },
	  {174, 55 }
  },
  {
 // Key Lights (99) - LED_FLAG_KEYLIGHT, LED_FLAG_MODIFIER
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		1, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 1, 4, 4, 4, 4, 4,
		1, 1, 1, 4, 1, 1, 4, 4, 4, 4, 4,

// Underglow/border LEDs (64) - LED_FLAG_UNDERGLOW
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2,

 // NCS Indicators (3)
		0, 0, 0
  }
};

#ifdef USB_LED_INDICATOR_ENABLE
void rgb_matrix_indicators_kb(void)
{
  led_matrix_indicators();
}
#endif // USB_LED_INDICATOR_ENABLE

#endif
