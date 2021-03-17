/*
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"


typedef enum uint8_t
{
    RED    = 1,
    GREEN  = 2,
    BLUE   = 3,
    YELLOW = 4,
    PURPLE = 5,
    CYAN   = 6,
    WHITE  = 7,
} Colours;



void set_rgb_led_colour(Colours Colour);







int main(void)
{
    // Configure board
    bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS);

    // Toggle LEDs
    while (true)
    {   
    

        bsp_board_led_on(BSP_BOARD_LED_0);
        nrf_delay_us(500000);
        bsp_board_led_off(BSP_BOARD_LED_0);

        set_rgb_led_colour(RED);
        nrf_delay_us(500000);
        set_rgb_led_colour(GREEN);
        nrf_delay_us(500000);
        set_rgb_led_colour(BLUE);
        nrf_delay_us(500000);
        set_rgb_led_colour(YELLOW);
        nrf_delay_us(500000);
        set_rgb_led_colour(PURPLE);
        nrf_delay_us(500000);
        set_rgb_led_colour(CYAN);
        nrf_delay_us(500000);
        set_rgb_led_colour(WHITE);
        nrf_delay_us(1000000);






        //if(bsp_board_button_state_get(BSP_BOARD_BUTTON_0)) // (0, 1, 2, 3) LEDs
        //{
        //    bsp_board_led_on(BSP_BOARD_LED_2);
        //    //BSP_BOARD_LED_0
        //}
        //else
        //{
        //    bsp_board_led_off(BSP_BOARD_LED_2);
        //}
    }
}


// Function for setting the RGB LED (doesnt include the )
void set_rgb_led_colour(Colours colour)
{
    // Disable all LED's as quicker than checking each one
    bsp_board_led_off(BSP_BOARD_LED_1);
    bsp_board_led_off(BSP_BOARD_LED_2);
    bsp_board_led_off(BSP_BOARD_LED_3);


    switch(colour)
    {
        case RED:
            bsp_board_led_on(BSP_BOARD_LED_1);
            break;

        case GREEN:
            bsp_board_led_on(BSP_BOARD_LED_2);
            break;

        case BLUE:
            bsp_board_led_on(BSP_BOARD_LED_3);
            break;

        case YELLOW: // R + G
            bsp_board_led_on(BSP_BOARD_LED_1);
            bsp_board_led_on(BSP_BOARD_LED_2);
            break;

        case PURPLE: // R + B
            bsp_board_led_on(BSP_BOARD_LED_1);
            bsp_board_led_on(BSP_BOARD_LED_3);
            break;

        case CYAN: // G + B
            bsp_board_led_on(BSP_BOARD_LED_2);
            bsp_board_led_on(BSP_BOARD_LED_3);
            break;

        case WHITE: // R + G + B
            bsp_board_led_on(BSP_BOARD_LED_1);
            bsp_board_led_on(BSP_BOARD_LED_2);
            bsp_board_led_on(BSP_BOARD_LED_3);
            break;

        default:
            break;

    }
}
