// delay.cpp

void delay(int count){
    // volatile so that the compiler doesn't optimise it out
    volatile int i;
    for (i = 0; i < count; i++){
    }
}
