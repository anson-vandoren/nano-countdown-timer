#define LED_1 3
#define LED_2 4

#define SHIFT_DISPLAY

#ifndef SHIFT_DISPLAY  // timer has discrete pins

#define BAR_0 2
#define BAR_1 5
#define BAR_2 6
#define BAR_3 7
#define BAR_4 8
#define BAR_5 9
#define BAR_6 10
#define BAR_7 11

#endif

#ifdef SHIFT_DISPLAY // timer uses a shift register

#define SER_CLK 5
#define SER_DATA 6

#endif