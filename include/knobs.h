#ifndef knobs
#define knobs

#include <algorithm>
#include <STM32FreeRTOS.h>

extern SemaphoreHandle_t keyArrayMutex;
extern volatile uint8_t keyArray[7];

class Knob{

    public:

        int id;

        int value;
        int max_value;
        int min_value;

        int a_current;
        int b_current;
        int a_prev;
        int b_prev;
        int direction;

        Knob(int start, int max, int min, int identifier) {
            id = identifier;
            value = start;
            max_value = max;
            min_value = min;
            a_prev = 0;
            b_prev = 0;
            direction;
        }

        int clamp(int val){
            return std::max(std::min(val, max_value), min_value);
        }

        void set_value(int val) {
            __atomic_store_n(&value, val, __ATOMIC_RELAXED);
        }

        void read_knob() {


            xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
            switch(id) {
                case 0:
                    a_current = keyArray[4] & 2;
                    b_current = keyArray[4] & 1;
                    break;
                case 1:
                    a_current = (keyArray[4] & 8) >> 2;
                    b_current = (keyArray[4] & 4) >> 2;

                    break;
                case 2:
                    a_current = keyArray[3] & 2;
                    b_current = keyArray[3] & 1;
                    break;
                case 3:
                    a_current = (keyArray[3] & 8) >> 2;
                    b_current = (keyArray[3] & 4) >> 2;
                    break;
                default:
                    break;
            }
            xSemaphoreGive(keyArrayMutex);

            if(a_current == a_prev) {
                direction = 0;
            }

            else {

                if( (b_current | a_current) == 0 || (b_current | a_current) == 3) {
                    direction = -1;
                }
                else if( (b_current  | a_current) == 1 || (b_current | a_current) == 2) {
                    direction = 1;
                }
            }

            a_prev = a_current;
            b_prev = b_current;

            set_value(clamp(value+ 1*direction));
        }
};


#endif