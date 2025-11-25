#include "stm32f405xx.h"

#include <stdint.h>
#include "lcd.h"
//--------------------------------ULTRASONIC-------------------------------
//------------------------------------------------------------------------
//------------------------------------------------------------------------

#define TRIG_PIN    8   // PA8 (Trigger) 41

#define ECHO_PIN    9   // PA9 (Echo) 42

#define TRIG_PIN2    8  // PC8 (Trigger) 39

#define ECHO_PIN2    10  // PC10 (Echo) 51

#define TRIG_PIN3    11  // PC11 (Trigger)

#define ECHO_PIN3    12  // PC12 (Echo)

#define TRIG_PIN4    10  // PB10 (Trigger)

#define ECHO_PIN4    11  // PB11 (Echo)



//#define LED_PIN     10   // PB10 (Onboard LED)-pa8,9 and pc8,9

//#define LED_PIN2     4   // PA4 (Onboard LED)-pc11,12



void Delay_us(uint32_t us) {
    uint32_t start = TIM2->CNT;
    while ((uint32_t)(TIM2->CNT - start) < us);
}


void GPIO_Init(void) {
    // Enable GPIO clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // Enable clock for GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // Enable clock for GPIOB
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // Enable clock for GPIOC

    // ---------------- GPIOA ----------------

    // PA8 (Trig1) - Output Push-Pull
    GPIOA->MODER  |=  (1 << (TRIG_PIN * 2));   // Set PA8 as output (01)
    GPIOA->OTYPER &= ~(1 << TRIG_PIN);         // Push-Pull

    // PA9 (Echo1) - Input (No Pull-up/down)
    GPIOA->MODER  &= ~(3 << (ECHO_PIN * 2));   // Set PA9 as input (00)

    // ---------------- GPIOC ----------------

    // PC8 (Trig2) - Output Push-Pull
    GPIOC->MODER  |=  (1 << (TRIG_PIN2 * 2));  // Set PC8 as output (01)
    GPIOC->OTYPER &= ~(1 << TRIG_PIN2);        // Push-Pull

    // PC9 (Echo2) - Input (No Pull-up/down)
    GPIOC->MODER  &= ~(3 << (ECHO_PIN2 * 2));  // Set PC9 as input (00)

    // PC11 (Trig3) - Output Push-Pull
    GPIOC->MODER  |=  (1 << (TRIG_PIN3 * 2));  // Set PC11 as output (01)
    GPIOC->OTYPER &= ~(1 << TRIG_PIN3);        // Push-Pull

    // PC12 (Echo3) - Input (No Pull-up/down)
    GPIOC->MODER  &= ~(3 << (ECHO_PIN3 * 2));  // Set PC12 as input (00)

    // ---------------- GPIOB ----------------

    // PB10 (Trig4) - Output Push-Pull
    GPIOB->MODER  |=  (1 << (TRIG_PIN4 * 2));  // Set PB10 as output (01)
    GPIOB->OTYPER &= ~(1 << TRIG_PIN4);        // Push-Pull

    // PB11 (Echo4) - Input (No Pull-up/down)
    GPIOB->MODER  &= ~(3 << (ECHO_PIN4 * 2));  // Set PB11 as input (00)


   // -----------------LEDS-------------------
    // PB10 (LED1) - Output Push-Pull
//    GPIOB->MODER  |=  (1 << (LED_PIN * 2));    // Set PA5 as output (01)
//    GPIOB->OTYPER &= ~(1 << LED_PIN);          // Push-Pull

    // Set PB7 as input
    GPIOB->MODER &= ~(3 << (7 * 2)); // 00 for input mode

    // Optional: enable pull-up resistor
    GPIOB->PUPDR |= (1 << (7 * 2));  // 01 for pull-up

}


void TIM2_Init(void) {

    // Enable TIM2 clock

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Configure TIM2 for 1µs resolution (84MHz / 84 = 1MHz)

    TIM2->PSC = 84 - 1;       // Prescaler

    TIM2->ARR = 0xFFFFFFFF;   // Auto-reload max

    TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2

}

void Trigger_Pulse(GPIO_TypeDef *port, uint8_t trig_pin) {
    port->ODR |= (1 << trig_pin);
    Delay_us(10);
    port->ODR &= ~(1 << trig_pin);
}

uint8_t Is_Object_Detected(GPIO_TypeDef *trig_port, uint8_t trig_pin,
                           GPIO_TypeDef *echo_port, uint8_t echo_pin) {
    uint32_t start_time = 0, end_time = 0, pulse_duration = 0;
    const uint32_t MAX_ECHO_WAIT_US = 60000;
    uint32_t timeout_start;

    Trigger_Pulse(trig_port, trig_pin);

    timeout_start = TIM2->CNT;
    while (!(echo_port->IDR & (1 << echo_pin))) {
        if ((TIM2->CNT - timeout_start) > MAX_ECHO_WAIT_US) return 0;
    }
    start_time = TIM2->CNT;

    timeout_start = TIM2->CNT;
    while (echo_port->IDR & (1 << echo_pin)) {
        if ((TIM2->CNT - timeout_start) > MAX_ECHO_WAIT_US) return 0;
    }
    end_time = TIM2->CNT;

    pulse_duration = (end_time >= start_time)
                     ? (end_time - start_time)
                     : (0xFFFFFFFF - start_time + end_time);

    return (pulse_duration < 875);
}




//----------------------------KEYPAD--------------------------------
//-----------------------------------------------------------------
const char keymap[4][4] = {
	    {'1', '2', '3', 'A'},
	    {'4', '5', '6', 'B'},
	    {'7', '8', '9', 'C'},
	    {'*', '0', '#', 'D'}
	};

void delayms(uint32_t dly)
{
  uint32_t i,j=0;
  for(i=0;i<dly;i++)
  for(j=0;j<16000;j++);
}

void gpio_keypad_init (void){
	RCC->AHB1ENR |= (1 << 2); // port c

	//set rows as output PC0-PC3
	GPIOC->MODER |=  (0x1 <<0);
	GPIOC->MODER |=  (0x1 <<2);
	GPIOC->MODER |=  (0x1 <<4);
	GPIOC->MODER |=  (0x1 <<6);

	//set columns to input PC4-PC7
	GPIOC->MODER &=  ~(0x0 <<8);
	GPIOC->MODER &=  ~(0x0 <<10);
	GPIOC->MODER &=  ~(0x0 <<12);
	GPIOC->MODER &=  ~(0x0 <<14);



	    // Enable pull-up resistors for PC4–PC7
	   GPIOC->PUPDR &= ~(0xFF << 8);     // Clear
	    GPIOC->PUPDR |=  (0x55 << 8);     // Pull-up (01)
}




char scan_keypad(void) {
    for (int row = 0; row < 4; row++) {
        GPIOC->ODR |= 0x0F;              // Set all rows HIGH
        GPIOC->ODR &= ~(1 << row);       // Pull current row LOW

        for (volatile int d = 0; d < 1000; d++); // short delay

        for (int col = 0; col < 4; col++) {
            if ((GPIOC->IDR & (1 << (col + 4))) == 0) {
                return keymap[row][col]; // Return mapped key
            }
        }
    }
    return 0; // No key pressed
}


//--------------------------------LED TIMER--------------------------------
//------------------------------------------------------------------------
// TM1637 bit delay
void TM1637_delay() {
	for (volatile int i = 0; i < 100; i++);
}

// Enable GPIOA and GPIOB, and configure pins
void gpio_init_timer(void) {
	RCC->AHB1ENR |= (1 << 0); // Enable GPIOA
	RCC->AHB1ENR |= (1 << 1); // Enable GPIOB

	// Traffic LEDs on GPIOA: PA2, PA3, PA4, PA5, PA6, PA7
	GPIOA->MODER &= ~((3 << 4) | (3 << 6) | (3 << 8) | (3 << 10) | (3 << 12) | (3 << 14));
	GPIOA->MODER |=  ((1 << 4) | (1 << 6) | (1 << 8) | (1 << 10) | (1 << 12) | (1 << 14));

	// TM1637 1: PB8 = DIO, PB9 = CLK
	// TM1637 2: PB4 = DIO, PB5 = CLK
	GPIOB->MODER &= ~((3 << 16) | (3 << 18) | (3 << 8) | (3 << 10));
	GPIOB->MODER |=  ((1 << 16) | (1 << 18) | (1 << 8) | (1 << 10));
}

// Segment lookup table
uint8_t segment_digit[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

// TM1637 abstraction using dio_pin and clk_pin
void set_pin(int pin, int value) {
	if (value)
		GPIOB->ODR |= (1 << pin);
	else
		GPIOB->ODR &= ~(1 << pin);
}

void TM1637_start(int dio_pin, int clk_pin) {
	set_pin(clk_pin, 1);
	set_pin(dio_pin, 1);
	TM1637_delay();
	set_pin(dio_pin, 0);
	TM1637_delay();
}

void TM1637_stop(int dio_pin, int clk_pin) {
	set_pin(clk_pin, 0);
	TM1637_delay();
	set_pin(dio_pin, 0);
	TM1637_delay();
	set_pin(clk_pin, 1);
	TM1637_delay();
	set_pin(dio_pin, 1);
	TM1637_delay();
}

void TM1637_write_byte(int dio_pin, int clk_pin, uint8_t b) {
	for (int i = 0; i < 8; i++) {
		set_pin(clk_pin, 0);
		set_pin(dio_pin, b & 0x01);
		TM1637_delay();
		set_pin(clk_pin, 1);
		TM1637_delay();
		b >>= 1;
	}
	// Skip ACK
	set_pin(clk_pin, 0);
	TM1637_delay();
	set_pin(clk_pin, 1);
	TM1637_delay();
	set_pin(clk_pin, 0);
	TM1637_delay();
}

void TM1637_display_digit(int dio_pin, int clk_pin, int pos, int digit) {
	TM1637_start(dio_pin, clk_pin);
	TM1637_write_byte(dio_pin, clk_pin, 0x44); // Command 1: set data
	TM1637_stop(dio_pin, clk_pin);

	TM1637_start(dio_pin, clk_pin);
	TM1637_write_byte(dio_pin, clk_pin, 0xC0 | pos); // Command 2: set address
	TM1637_write_byte(dio_pin, clk_pin, segment_digit[digit]);
	TM1637_stop(dio_pin, clk_pin);

	TM1637_start(dio_pin, clk_pin);
	TM1637_write_byte(dio_pin, clk_pin, 0x88); // Command 3: control display
	TM1637_stop(dio_pin, clk_pin);
}

// Timer delay using TIM3
void TIM3_init(void) {
	RCC->APB1ENR |= (1 << 1); // Enable TIM3
	TIM3->PSC = 16000 - 1;    // 1ms tick
	TIM3->ARR = 1000 - 1;     // 1s delay
	TIM3->CNT = 0;
	TIM3->SR = 0;
}

void delay_1s(void) {
	TIM3->CNT = 0;
	TIM3->SR &= ~1;
	TIM3->CR1 |= 1;

	while (!(TIM3->SR & 1));
	TIM3->SR &= ~1;
	TIM3->CR1 &= ~1;
}


void starting_show(void) {
		TM1637_display_digit(8,9,0, 0); // Tens
		TM1637_display_digit(8,9,1, 0); // Units
		TM1637_display_digit(8,9,2, 0);
		TM1637_display_digit(8,9,3, 0);

		TM1637_display_digit(4,5,0,0); // Tens
		TM1637_display_digit(4,5,1,0); // Units
		TM1637_display_digit(4,5,2,0);
		TM1637_display_digit(4,5,3,0);

		//delay_1s();
}

void red_show_countdown(int seconds) {
	for (int i = seconds; i > 0; i--) {
		TM1637_display_digit(8,9,0, i / 10); // Tens
		TM1637_display_digit(8,9,1, i % 10); // Units
		TM1637_display_digit(8,9,2, 0);
		TM1637_display_digit(8,9,3, 0);

		TM1637_display_digit(4,5,0, i / 10); // Tens
		TM1637_display_digit(4,5,1, i % 10); // Units
		TM1637_display_digit(4,5,2, 0);
		TM1637_display_digit(4,5,3, 0);

		delay_1s();
	}
}

// Unified countdown display on both TM1637 modules and LEDs
void show_countdown(
	int red_seconds, int green_seconds, int yellow_seconds,
	int red_pin_side1, int yellow_pin_side1, int green_pin_side1,
	int red_pin_side2, int yellow_pin_side2, int green_pin_side2,
	int dio1, int clk1, int dio2, int clk2,int show_red_on_display1)
{

	for (int i = 0; i < red_seconds; i++) {
		int rem = green_seconds + yellow_seconds - i;
		int red_rem = red_seconds - i;

		// Side 1 RED display
		if (show_red_on_display1) {
			TM1637_display_digit(dio1, clk1, 0, red_rem / 10);
			TM1637_display_digit(dio1, clk1, 1, red_rem % 10);
			TM1637_display_digit(dio1, clk1, 2, 0);
			TM1637_display_digit(dio1, clk1, 3, 0);
		} else {
			TM1637_display_digit(dio2, clk2, 0, red_rem / 10);
			TM1637_display_digit(dio2, clk2, 1, red_rem % 10);
			TM1637_display_digit(dio2, clk2, 2, 0);
			TM1637_display_digit(dio2, clk2, 3, 0);
		}

		// Clear all LEDs
		GPIOA->ODR &= ~((1 << red_pin_side1) | (1 << yellow_pin_side1) | (1 << green_pin_side1) |
		                (1 << red_pin_side2) | (1 << yellow_pin_side2) | (1 << green_pin_side2));

		// Turn on RED for side 1
		GPIOA->ODR |= (1 << red_pin_side1);

		// Side 2: show GREEN or YELLOW countdown
		if (rem > yellow_seconds) {
			int g_rem = rem - yellow_seconds;
			TM1637_display_digit(dio2, clk2, 0, g_rem / 10);
			TM1637_display_digit(dio2, clk2, 1, g_rem % 10);
			GPIOA->ODR |= (1 << green_pin_side2);
		} else if (rem > 0) {
			TM1637_display_digit(dio2, clk2, 0, rem / 10);
			TM1637_display_digit(dio2, clk2, 1, rem % 10);
			GPIOA->ODR |= (1 << yellow_pin_side2);
		} else {
			TM1637_display_digit(dio2, clk2, 0, 0);
			TM1637_display_digit(dio2, clk2, 1, 0);
		}

		TM1637_display_digit(dio2, clk2, 2, 0);
		TM1637_display_digit(dio2, clk2, 3, 0);

		delay_1s();
	}

	GPIOA->ODR &= ~((1 << red_pin_side1) | (1 << yellow_pin_side1) | (1 << green_pin_side1) |
	                (1 << red_pin_side2) | (1 << yellow_pin_side2) | (1 << green_pin_side2));


}



int main(void) {

	LcdInit();

	gpio_init_timer();
	TIM3_init();

	gpio_keypad_init();
    GPIO_Init();

    TIM2_Init();



    char passkey[5] = {0};
   	 const char password[] = "258A";

   	 GPIOA->ODR &= ~((1 << 2) |(1 << 3) |(1 << 4) |(1 << 5) |(1 << 6) | (1 << 7));
   	 starting_show();
   	 LcdFxn(0, 0x01); // Clear screen
   	 lprint(0x80, "Welcome");

   	 while (1) {
   	        char key = scan_keypad();
   	        if (key == '*') {
   	            LcdFxn(0, 0x01); // Clear LCD
   	            lprint(0x80, "Enter Passkey:");
   	            memset(passkey, 0, sizeof(passkey));
   	            int i = 0;

   	            while (1) {
   	                key = scan_keypad();
   	                if (key) {
   	                    if (key == '#') {
   	                        passkey[i] = '\0'; // Null-terminate before checking
   	                        LcdFxn(0, 0x01);   // Clear LCD

   	                        if (strcmp(passkey, password) == 0) {
   	                            lprint(0x80, "Please come in");
   	                        } else {
   	                            lprint(0x80, "Wrong passkey");
   	                        }

   	                        delayms(300); // Show result for 1 second
   	                        LcdFxn(0, 0x01);
   	                        lprint(0x80, "Welcome");
   	                        break;  // Exit password entry
   	                    }

   	                    if (i < 4) {
   	                        passkey[i++] = key;
   	                        lprint(0xC0 + i - 1, "*");
   	                    }
   	                    while (scan_keypad()); // Wait for key release
   	                    delayms(50);
   	                }
   	            }
   	            if (strcmp(passkey, password) == 0) {
   	            	LcdFxn(0, 0x01);
   	                 break;
   	                  }
   	        }
   	    }


   	GPIOA->ODR = 0;
   	GPIOA->ODR |= (1 << 2) | (1 << 5); // Initial RED LEDs
   	red_show_countdown(10);
   	GPIOA->ODR = 0;
   	starting_show();

    while (1) {
    	uint8_t c1=0,c2=0;


    	// Check if PB7 is pressed (active LOW)
    	if (!(GPIOB->IDR & GPIO_IDR_ID7)) {
    	    delayms(50); // debounce press

    	    if (!(GPIOB->IDR & GPIO_IDR_ID7)) { // still pressed
    	        // --- Your special routine (runs once) ---
    	        GPIOA->ODR &= ~((1 << 2) |(1 << 3) |(1 << 4) |(1 << 5) |(1 << 6) | (1 << 7));
    	        starting_show();
    	        LcdFxn(0, 0x01); // Clear screen
    	        lprint(0x80, "Powering Off.");
    	        GPIOA->ODR |= (1 << 2) | (1 << 5); // Initial RED LEDs
    	        lprint(0xC0, "Wait for rebooting.");
    	        red_show_countdown(10);
    	        GPIOA->ODR = 0;

    	        // --- Wait until button is released ---
    	        while (!(GPIOB->IDR & GPIO_IDR_ID7));  // wait for release
    	        delayms(50); // debounce release
    	    }
    	}



        uint8_t obj1 = Is_Object_Detected(GPIOA, TRIG_PIN, GPIOA, ECHO_PIN);     // PA8/PA9
        Delay_us(100);
        uint8_t obj2 = Is_Object_Detected(GPIOC, TRIG_PIN2, GPIOC, ECHO_PIN2);   // PC8/PC10
        Delay_us(100);
        uint8_t obj3 = Is_Object_Detected(GPIOC, TRIG_PIN3, GPIOC, ECHO_PIN3);   // PC11/PC12
        Delay_us(100);
        uint8_t obj4 = Is_Object_Detected(GPIOB, TRIG_PIN4, GPIOB, ECHO_PIN4);   // PB10/PB11
        Delay_us(100);

        if (obj1) c1++;
        if (obj2)c1++;
        if (obj3) c2++;
        if (obj4)c2++;

        if (c1>c2) {
           // GPIOB->ODR |= (1 << LED_PIN);     // LED1 ON
            LcdFxn(0, 0x01);
            lprint(0x80, "side1 Vec detected");
	        show_countdown(10, 8, 2, 2, 3, 4, 5, 6, 7, 8, 9, 4, 5, 1);

        }
        else if (c2>c1) {
           // GPIOB->ODR &= ~(1 << LED_PIN);    // LED1 OFF
            LcdFxn(0, 0x01);
            lprint(0x80, "side2 Vec detected");
            show_countdown(10, 8, 2, 5, 6, 7, 2, 3, 4, 4, 5, 8, 9, 1);
        }

        else {
  	  LcdFxn(0, 0x01);
  	  lprint(0x80, "no priority.");
  	  show_countdown(10, 8, 2, 2, 3, 4, 5, 6, 7, 8, 9, 4, 5, 1);
  	  show_countdown(10, 8, 2, 5, 6, 7, 2, 3, 4, 4, 5, 8, 9, 1);
        }

    }
}