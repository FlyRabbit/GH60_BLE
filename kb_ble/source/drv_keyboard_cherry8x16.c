/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision$
 */
 
 #include "drv_keyboard_matrix.h"
 #include "io_cfg.h"
 
#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"

#define CHERRY8x16_NUM_OF_COLUMNS IO_CHERRY8x16_NUM_COLUMNS //!< Number of columns in the keyboard matrix
#define CHERRY8x16_NUM_OF_ROWS    IO_CHERRY8x16_ROWS        //!< Number of rows in the keyboard matrix
#define CHERRY8x16_COLUMN_PORT    IO_CHERRY8x16_COLUMN_PORT //!< Bitmask of column pins
#define CHERRY8x16_ROW_PORT       IO_CHERRY8x16_ROW_PORT    //!< Bitmask of row pins

#define MODIFIER_HID_START         0xE0
#define MODIFIER_HID_END           0xE7
#define MODIFIER_LEFT_CONTROL_HID  0xE0
#define MODIFER_RIGHT_CONTROL_HID  0xE4

extern bool            pair_btn_pressed;

/** Table describing relationship between column number and column pin */
static const uint8_t m_column_to_pin_map[CHERRY8x16_NUM_OF_COLUMNS] = 
{
  10, 24, 17, 19, 7
};

/** Table containing the mapping between the key matrix and the HID Usage codes for each key. */
static const uint8_t matrix_lookup[CHERRY8x16_NUM_OF_COLUMNS*CHERRY8x16_NUM_OF_ROWS] = 
{
  0x35, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x2D, 0x2E, 0x2A,
	0x2B, 0x14, 0x1A, 0x08, 0x15, 0x17, 0x1C, 0x18, 0x0C, 0x12, 0x13, 0x2F, 0x30, 0x31, 
	0x39, 0x04, 0x16, 0x07, 0x09, 0x0A, 0x0B, 0x0D, 0x0E, 0x0F, 0x33, 0x34, 0x00, 0x28, 
	0xE1, 0x1D, 0x1B, 0x06, 0x19, 0x05, 0x11, 0x10, 0x36, 0x37, 0x38, 0x00, 0x00, 0xE5, 
	0xE0, 0xE3, 0xE2, 0xFF, 0x00, 0x00, 0x2C, 0x00, 0x00, 0x00, 0x4B, 0x4E, 0x29, 0x4C
};

static uint16_t cherry8x16_row_read(void);

void drv_keyboard_init(void)
{    
    int i;
    
    for (i = 32; i >= 0; --i)
    {
        if (CHERRY8x16_ROW_PORT & (1 << i))
        {
            NRF_GPIO->PIN_CNF[i] =
                (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)    |
                (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)  |
                (GPIO_PIN_CNF_PULL_Pulldown  << GPIO_PIN_CNF_PULL_Pos)   |
                (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)  |
                (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
        }
    }
    
    NRF_GPIO->DIRSET = CHERRY8x16_COLUMN_PORT;
    NRF_GPIO->OUTCLR = CHERRY8x16_COLUMN_PORT;
}

void drv_keyboard_sense_enable(bool p_enable)
{
    uint32_t sense;
    int i;
    
    NRF_GPIO->DIRSET = CHERRY8x16_COLUMN_PORT;
    
    if (p_enable)
    {
        sense = GPIO_PIN_CNF_SENSE_High;
        // Columns high
        NRF_GPIO->OUTSET = CHERRY8x16_COLUMN_PORT;
    }
    else
    {
        sense = GPIO_PIN_CNF_SENSE_Disabled;
        // Columns low
        NRF_GPIO->OUTCLR = CHERRY8x16_COLUMN_PORT;
    }
    
    // Configuring rows
    for (i = 32; i >= 0; --i)
    {
        if (CHERRY8x16_ROW_PORT & (1 << i))
        {
            NRF_GPIO->PIN_CNF[i] =
                (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)    |
                (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)  |
                (GPIO_PIN_CNF_PULL_Pulldown  << GPIO_PIN_CNF_PULL_Pos)   |
                (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)  |
                (sense                       << GPIO_PIN_CNF_SENSE_Pos);
        }
    }
}

bool drv_keyboard_keys_held(void)
{
    uint32_t port;
    
    NRF_GPIO->DIRSET = IO_CHERRY8x16_COLUMN_PORT;
    NRF_GPIO->OUTSET = IO_CHERRY8x16_COLUMN_PORT;
    for (port = 0; port < 5; ++port)
    {
        __NOP();
    }
    port = (NRF_GPIO->IN & IO_CHERRY8x16_ROW_PORT);
    NRF_GPIO->OUTCLR = IO_CHERRY8x16_COLUMN_PORT;    
    
    return (port != 0);
}

void drv_keyboard_wakeup_prepare(void)
{
    int i;
    
    for (i = 32; i >= 0; --i)
    {
        if (CHERRY8x16_ROW_PORT & (1 << i))
        {
            NRF_GPIO->PIN_CNF[i] =
                (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)    |
                (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)  |
                (GPIO_PIN_CNF_PULL_Pulldown  << GPIO_PIN_CNF_PULL_Pos)   |
                (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)  |
                (GPIO_PIN_CNF_SENSE_Disabled     << GPIO_PIN_CNF_SENSE_Pos);
        }
    }
    NRF_GPIO->DIRSET = CHERRY8x16_COLUMN_PORT;
    NRF_GPIO->OUTSET = CHERRY8x16_COLUMN_PORT;
    for (i = 32; i >= 0; --i)
    {
        if (CHERRY8x16_ROW_PORT & (1 << i))
        {
            NRF_GPIO->PIN_CNF[i] =
                (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos)    |
                (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos)  |
                (GPIO_PIN_CNF_PULL_Pulldown  << GPIO_PIN_CNF_PULL_Pos)   |
                (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos)  |
                (GPIO_PIN_CNF_SENSE_High     << GPIO_PIN_CNF_SENSE_Pos);
        }
    }
    

}


/**
 * Reads and returns keyboard matrix row state.
 *
 * @return uint8_t Row state
 */
static uint16_t cherry8x16_row_read(void)
{
  uint32_t port_val = 0;
  uint16_t row_port = 0;
  
  port_val = NRF_GPIO->IN;
  row_port = ((port_val & GPIO_IN_PIN0_Msk) >> (GPIO_IN_PIN0_Pos - 0)) | 
             ((port_val & GPIO_IN_PIN1_Msk) >> (GPIO_IN_PIN1_Pos - 1)) | 
             ((port_val & GPIO_IN_PIN2_Msk) >> (GPIO_IN_PIN2_Pos - 2)) |
             ((port_val & GPIO_IN_PIN5_Msk) >> (GPIO_IN_PIN5_Pos - 3)) |
             ((port_val & GPIO_IN_PIN8_Msk) >> (GPIO_IN_PIN8_Pos - 4)) |
             ((port_val & GPIO_IN_PIN11_Msk) >> (GPIO_IN_PIN11_Pos - 5)) |
             ((port_val & GPIO_IN_PIN12_Msk)  >> (GPIO_IN_PIN12_Pos  - 6)) |
             ((port_val & GPIO_IN_PIN14_Msk) >> (GPIO_IN_PIN14_Pos - 7)) |
							((port_val & GPIO_IN_PIN25_Msk) >> (GPIO_IN_PIN25_Pos - 8)) |
							((port_val & GPIO_IN_PIN23_Msk) >> (GPIO_IN_PIN23_Pos - 9)) |
							((port_val & GPIO_IN_PIN21_Msk) >> (GPIO_IN_PIN21_Pos - 10)) |
							((port_val & GPIO_IN_PIN28_Msk) >> (GPIO_IN_PIN28_Pos-11)) |
							((port_val & GPIO_IN_PIN29_Msk) >> (GPIO_IN_PIN29_Pos-12)) |
							((port_val & GPIO_IN_PIN30_Msk) >> (GPIO_IN_PIN30_Pos-13));	
  
  return row_port;
}

/**
 * Sets matrix column high.
 *
 * @param[in] column Which column to set [0, 1, ..., 15]
 */
__INLINE static void cherry8x16_column_set(uint_fast8_t column)
{
    uint32_t outset_val = (1 << m_column_to_pin_map[column]);
    NRF_GPIO->OUTSET = outset_val;
}

/**
 * Reads keyboard matrix state and stores pressed keys to an array.
 *
 * This function resolves keys from the matrix and finds their corresponding HID usage codes
 * If there are any ghost key conditions the packet will be discarded
 * @param pressed_keys Array holding pressed keys. Must be at least CHERRY8x16_MAX_NUM_OF_PRESSED_KEYS in size.
 * @param number_of_pressed_keys Pointer to variable where number of pressed keys will be stored.
 * @return 
 * @retval true If no keys were blocking each other.
 * @retval false If some keys were blocking each other o rno key is pressed.
 */
bool drv_keyboard_keymatrix_read(uint8_t *pressed_keys, uint8_t *number_of_pressed_keys)
{
    uint_fast16_t row_state[CHERRY8x16_NUM_OF_COLUMNS];
    uint_fast8_t blocking_mask = 0;
    uint_fast8_t column;
    uint_fast8_t row;

    *number_of_pressed_keys = 0;

    for (column = CHERRY8x16_NUM_OF_COLUMNS; column--;)
    {
        // Need to set all columns zero before pulling up next column (otherwise we read ghost keys due to capacitance from previous state) 
        NRF_GPIO->DIRCLR = CHERRY8x16_COLUMN_PORT;   // Make column pins as input
        NRF_GPIO->OUTCLR = CHERRY8x16_COLUMN_PORT;   // Pull down column pins to zero
        NRF_GPIO->DIRSET = CHERRY8x16_COLUMN_PORT;   // Make all pins as outputs so the pull down value propagates to keypad matrix

        // Now set all columns as input except the one driving
        NRF_GPIO->DIRCLR = CHERRY8x16_COLUMN_PORT;
        NRF_GPIO->DIRSET = (1UL << m_column_to_pin_map[column]);
        // drive column under test
        cherry8x16_column_set(column);
        __NOP(); // Needed for -O3 and "Optimize for Time" optimizations
        row_state[column] = cherry8x16_row_read();

        // Check if any keys are pressed
        if (row_state[column] != 0)
        {
            uint_fast8_t detected_keypresses_on_column = 0;

            // Loop through rows, check for active rows and add pressed keys to the array
            for (row = CHERRY8x16_NUM_OF_ROWS; row--;)
            {
                if (row_state[column] & (1U << row))
                {
                    if (*number_of_pressed_keys < KEYBOARD_MAX_NUM_OF_PRESSED_KEYS)
                    {
                        *pressed_keys = matrix_lookup[column*CHERRY8x16_NUM_OF_ROWS + row];
                        pressed_keys++;
                        (*number_of_pressed_keys)++;
                    }
                    detected_keypresses_on_column++;
                }
            }

            if (detected_keypresses_on_column > 1)
            {
                if (blocking_mask & row_state[column])
                {
                    // Cannot determine reliably all pressed keys, two or more keys are blocking each other.
                    NRF_GPIO->DIRSET = CHERRY8x16_COLUMN_PORT;
                    return false;
                }
            }
            blocking_mask |= row_state[column];
        }
    }

    NRF_GPIO->DIRSET = CHERRY8x16_COLUMN_PORT;
    return true;
}

bool drv_keyboard_remap_special_keys(uint8_t *keys, uint8_t modifier_keys, uint8_t number_of_keys)
{
    bool fn_key_set = false;
    uint_fast8_t i;
    
    for (i = 0; i < number_of_keys; ++i)
    {
        if (keys[i] == 0xFF)
        {
            fn_key_set = true;
            keys[i] = 0x00;
            break;
        }
    }
    
    if (!fn_key_set)
    {
        return fn_key_set;
    }
    
    // Check if Fn key is pressed along with any other modifier key (only usage now is Fn+Left_Ctrl = Right Ctrl)
    // So we modify the modifier byte if Fn+Left_Ctrl is pressed, HID for left_Ctrl = 0xE0 
    if ( modifier_keys & (1UL<<(MODIFIER_LEFT_CONTROL_HID-MODIFIER_HID_START)) )
    {
        modifier_keys &= ~(1UL<<(MODIFIER_LEFT_CONTROL_HID-MODIFIER_HID_START));
        modifier_keys |= (1UL<<(MODIFER_RIGHT_CONTROL_HID-MODIFIER_HID_START));
    }  

    for (i = 0; i < number_of_keys; i++)
    {
        pair_btn_pressed = false;
				switch (keys[i])
        {
          case 0x1A: //'W'
            keys[i] = 0x52; //up
            break;
          case 0x16: //'S'
            keys[i] = 0x51; //down
            break;
          case 0x04: //'A'
            keys[i] = 0x50; //left
            break;
          case 0x07: //'D'
            keys[i] = 0x4F; //right
            break;
          case 0x1E: //'K'
            keys[i] = 0x3A; //F1
            break;
          case 0x1F: //'L'
            keys[i] = 0x3B; //F2
            break;
          case 0x20: //''
            keys[i] = 0x3C; //F3
            break;
          case 0x21: //'Enter'
            keys[i] = 0x3D; //F4
            break;
          case 0x22: //'U'
            keys[i] = 0x3E; //F5
            break;
          case 0x23: //'I'
            keys[i] = 0x3F; //F6
            break;
          case 0x24: //'O'
            keys[i] = 0x40; //F7
            break;
          case 0x25: //'P'
            keys[i] = 0x41; //F8
            break;
          case 0x26: //'7'
            keys[i] = 0x42; //F9
            break;
          case 0x27: //'8'
            keys[i] = 0x43; //F10
            break;
          case 0x2D: //'9'
            keys[i] = 0x44; //F11
            break;
          case 0x2E: //'0'
            keys[i] = 0x45; //F12
            break;
					case 0x13:
						pair_btn_pressed = true;
						break;
          default:
            break; 
        }
    }
    return fn_key_set;
}
