// i2ctest2.cpp
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/i2c.h>

void setup_usart(void);
void setup_i2c(void);
void setup_gpio(void);
void i2c_write_byte(uint32_t i2c, uint8_t addr, uint8_t byte);

int main (void) {
    // Set system clock to 72MHz based on ext. 8MHz Xtal
    void rcc_clock_setup_in_hse_8mhz_out_72mhz(void); /*
        AHB  - 36MHz
        APB1 - 36MHz
        APB2 - 72MHz
        ADC  -  9MHz
        */
    setup_gpio();                           // Setup GPIO Ports & Functions
    setup_usart();                          // Setup & Enable USART1
    setup_i2c();                            // Setup & Enable I2C
    for (;;) {
        // Onboard LED will flash if program is looping correctly.
        gpio_toggle(GPIOC, GPIO13);
        // create a small delay by looping for a while
        for (int i = 0; i < 1000000; ++i) __asm("");

        usart_send_blocking(USART1, 'H');
        usart_send_blocking(USART1, 'i');
        usart_send_blocking(USART1, '!');
        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');

        i2c_write_byte(I2C1,0xB8,0x01);


    }
}

void setup_usart(void){
    // Setup USART1
    rcc_periph_clock_enable(RCC_GPIOA);     // Enable GPIO A Clock
    rcc_periph_clock_enable(RCC_USART1);    // Enable USART1 Clock
    rcc_periph_clock_enable(RCC_AFIO);      // Enable AF Clock

    // Setup GPIO A9 as USART1 TX
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

    //Setup USART1
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    usart_enable(USART1);                   // Enable USART1
}

void setup_i2c(void){
    // Setup I2C1

    rcc_periph_clock_enable(RCC_GPIOB);     // Enable GPIO B Clock
    // Setup PB8(SCL) & PB9(SDA) for I2C1
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO8);
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN, GPIO9);

    rcc_periph_clock_enable(RCC_I2C1);      // Enable I2C1 Clock
    rcc_periph_clock_enable(RCC_AFIO);      // Enable Alternate Function Clock

    i2c_peripheral_disable(I2C1);           // Disable I2C1 to make changes

    // Set FREQ in I2C1_CR2 based on APB1 (36MHz)
    i2c_set_clock_frequency(I2C1, I2C_CR2_FREQ_36MHZ);

    // Fast Mode, Duty = 2, I2C1_CCR = 0x801E
    i2c_set_fast_mode(I2C1);                // Set F/S bit (400kHz Fast Mode)
    i2c_set_ccr(I2C1, 0x1E);                // Set CCR Bits

    // Fast Mode - TRISE = 11 = 0x0B
    i2c_set_trise(I2C1,0x0B);               // Set Rise Time
    i2c_set_own_7bit_slave_address(I2C1, 0x42); // Set Address of this device
    i2c_peripheral_enable(I2C1);            // Enable I2C1
}

void setup_gpio(void){
    rcc_periph_clock_enable(RCC_GPIOC);     // Enable GPIO C Clock
    // Setup Pin C13 for onboard LED
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);


}

void i2c_write_byte(uint32_t i2c, uint8_t addr, uint8_t byte){
    uint32_t reg32 __attribute__((unused));

    i2c_send_start(i2c); // Send START

    // Wait until START is sent and for switchover to master mode.
    while (!((I2C_SR1(i2c) & I2C_SR1_SB) & (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))));

    i2c_send_7bit_address(i2c, addr, I2C_WRITE); // Send Address

    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR));     // Wait until Address Sent

    reg32 = I2C_SR2(i2c);                       // Read I2C_SR2 to clear ADDR Bit

    i2c_send_data(i2c, byte);                   // Send Data Byte
    while (!(I2C_SR1(i2c) & I2C_SR1_BTF));      // Wait for BTF Flag

    /* After the last byte we have to wait for TxE too. */
    while (!(I2C_SR1(i2c) & (I2C_SR1_BTF | I2C_SR1_TxE)));

    /* Send STOP condition. */
i2c_send_stop(i2c);
}

