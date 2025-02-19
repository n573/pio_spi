#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ad7699_spi.pio.h"
/**
 * Note, CNV outside the PIO did not work. it raised cnv before the end of the 16 SCK pulses
 */
#define KB0 14
#define KB1 25 //! GPIO25, PWMD on schematic
// #define Bob_SCK KB0
// #define Bob_MOSI KB1
// #define Bob_CNV KB1
#define Bob_CNV 9
#define Bob_SCK 10
#define Bob_MOSI 11
#define Bob_MISO 12
#define LED_R 6
#define LED_G 7
#define LED_B 8

#define ADC_CFG_DEFAULT 0b11110001001001 //! Overwrite bit is set! SEQ = 00 = sequencer disabled. Read-back CFG disabled. INTERNAL REF 4.096V
#define ADC_IN0 ADC_CFG_DEFAULT //! default is set up to read IN0
#define ADC_IN1 0b11110011001001
#define ADC_IN2 0b11110101001001
#define ADC_IN3 0b11110111001001
#define ADC_IN4 0b11111001001001
#define ADC_IN5 0b11111011001001
#define ADC_IN6 0b11111101001001
#define ADC_IN7 0b11111111001001
uint16_t adc_cfg[8] = {ADC_IN0, ADC_IN1, ADC_IN2, ADC_IN3, ADC_IN4, ADC_IN5, ADC_IN6, ADC_IN7};
#define TRANSLATE_TO_FL(value)    (float)(value*4.096/(1 << 16)) //! B943 has 16 bit adc - reference = 4.096V

// static uint16_t adc_data_store[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 
static uint32_t adc_data_store[8] = {0, 0, 0, 0, 0, 0, 0, 0}; 
static uint8_t adc_index = 1; //! Start at 1 because the config for IN0 has already been sent in initBobSPI()
uint8_t adc_flags[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t miso_index;

// Add these global variables at the top with other globals
static PIO pio = pio0;
#define NUM_SM_USING 1
// const uint sm = 0;
uint sm = 0;
 void ad7699_pio_init() {
    uint offset;
    pio_sm_config c;
    // Get first available PIO and state machine
    PIO pio = pio0;
    sm = pio_claim_unused_sm(pio, true); //! should give us sm = 0 -- confirmed
    offset = pio_add_program(pio, &ad7699_spi_program);

    for(int i=0; i<NUM_SM_USING; i++) {
        printf("SM %d @ offset %02x\n", sm, offset);
    }

    // Configure state machine
    // pio_sm_config c = ad7699_program_get_default_config(offset);
    c = ad7699_spi_program_get_default_config(offset);
    
    // Initialize pins to pio0 (accessable by all 4 SM on this PIO)
    pio_gpio_init(pio, Bob_SCK);
    pio_gpio_init(pio, Bob_MOSI);
    pio_gpio_init(pio, Bob_MISO);
    pio_gpio_init(pio, Bob_CNV);
    
    // Set up pin mapping -- MOSI is c[1] MISO is c[2]. SCK and CNV are c[0]
    // sm_config_set_set_pins(&c, Bob_SCK, 1);        // Set pins control SCK
    // sm_config_set_sideset_pins(&c, Bob_CNV);       // CNV pin
    sm_config_set_out_pins(&c, Bob_MOSI, 1);       // MOSI output
    sm_config_set_in_pins(&c, Bob_MISO);           // MISO input
    sm_config_set_sideset_pins(&c, Bob_SCK);        // Set pins control SCK
    sm_config_set_set_pins(&c, Bob_CNV, 1);       // CNV pin

    // Set pin directions
    // pio_sm_set_consecutive_pindirs(pio, sm, KB0, 1, true);  // set KB0 to output -- redundant if Bob_* is set to KB0/KB1
    // pio_sm_set_consecutive_pindirs(pio, sm, KB1, 1, true);  // set KB1 to output
    pio_sm_set_consecutive_pindirs(pio, sm, Bob_CNV, 3, true);   // SCK=10, CNV=9, MOSI=11, outputs (pins are sequential)
    // pio_sm_set_consecutive_pindirs(pio, sm, Bob_SCK, 2, true);   // SCK=10, MOSI=11, outputs (pins are sequential)
    pio_sm_set_consecutive_pindirs(pio, sm, Bob_MISO, 1, false); // MISO=12,  input
    //! @note these are set up individually because while SCK and MOSI are sequential, KB0 and KB1 are not so for debug purposes they need to be separate calls
    // pio_sm_set_consecutive_pindirs(pio, sm, Bob_SCK, 1, true);   
    // pio_sm_set_consecutive_pindirs(pio, sm, Bob_MOSI, 1, true);
    // pio_sm_set_consecutive_pindirs(pio, sm, Bob_CNV, 1, true);

    // Configure shift registers for 16-bit operation
    // sm_config_set_out_shift(&c[1], true, true, 16);   // Right shift, autopull, 16 bits
    // sm_config_set_in_shift(&c[2], true, true, 16);    // Right shift, autopush, 16 bits
    // sm_config_set_out_shift(&c, false, true, 14);   // Left shift, autopull, 14 bits b/c CFG is only 14 bits
    // sm_config_set_out_shift(&c, false, true, 16);   // Left shift, autopull, 16 bits. CFG is only 14 bits but we will shift it to make MSB @ bit 15
    sm_config_set_in_shift(&c, false, true, 16);    // Left shift, autopush, 16 bits
    sm_config_set_out_shift(&c, false, false, 14);   // Left shift, no-autopull, 14 bits b/c CFG is only 14 bits
    // sm_config_set_in_shift(&c, false, false, 16);    // Left shift, no-autopush, 16 bits

    // Set clock divider for reliable timing
    // float div = clock_get_hz(clk_sys) / (10000000); // 10MHz for testing
    // sm_config_set_clkdiv(&c, div);
    // printf("DIV = %f\n", div);
    sm_config_set_clkdiv(&c, 1.0f); //! "full-speed" --  125MHz
    // sm_config_set_clkdiv(&c, 1.25f); //! ~100MHz
    // sm_config_set_clkdiv(&c, 2.5f); //! ~50MHz

    // Clear FIFOs and restart state machines
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_clear_fifos(pio, sm); //! clear MOSI & MISO FIFO
    
    // pio_enable_sm_mask_in_sync(pio, sm_mask);
    // pio_restart_sm_mask(pio, sm_mask);
    // pio_set_sm_mask_enabled(pio, sm_mask, true);
    // Skip sync since we only have 1 SM
    pio_sm_restart(pio, sm);
    pio_sm_set_enabled(pio, sm, true);
    
}

// Modify bob_irq to remove initialization
bool bob_irq(repeating_timer_t* rt) {
    gpio_put(LED_G, 1);
    gpio_put(LED_B, 1);
    // adc_index = 0; //! @attention This is for debugging. Remove after functionality has been verified.
    // adc_index = 1; //! @attention This is for debugging. Remove after functionality has been verified.
    // adc_index = adc_index % 2; //! is 1 or 0. for testing
    miso_index = (adc_index + 6) % 8;
    
    // gpio_put(Bob_CNV, 0); //! Set CNV low to start conversion

    // Check if TX FIFO is full
    if (pio_sm_is_tx_fifo_full(pio, sm)) {
        gpio_put(LED_B, 0);
        pio_sm_drain_tx_fifo(pio, sm);
    }
    
    // Send configuration with blocking
    // pio_sm_put_blocking(pio, sm, (adc_cfg[adc_index]<<2)); //! Shift by 2 to align the MSB to a 16bit bit-string
    // pio_sm_put_blocking(pio, sm, (uint32_t)adc_cfg[adc_index]);
    pio_sm_put_blocking(pio, sm, (uint32_t)(adc_cfg[adc_index]<<18)); /// @attention  Aligns MSB with bit 31
    // printf("CFG %d: %d\n", adc_index, adc_cfg[adc_index]);

    // Check if RX FIFO has data
    if (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        // uint16_t result = pio_sm_get(pio, sm);
        // uint32_t result = pio_sm_get(pio, sm); //! a "word" is 32-bits
        uint32_t result = pio_sm_get_blocking(pio, sm); //! a "word" is 32-bits
        adc_data_store[miso_index] = result;
        adc_flags[miso_index] = 1;
        adc_flags[adc_index] = 0;
    } else {gpio_put(LED_B, 0);}
    // gpio_put(Bob_CNV, 1); //! Set CNV high to end conversion
    adc_index = (adc_index + 1) % 8;
    gpio_put(LED_G, 0);
    pio_sm_clear_fifos(pio, sm); //! clear FIFOs before next cycle
    return true;
}

void delay_idea(uint32_t count) {
    for(int i=0; i<count; i++) {
        //  gpio_put(KB0, !gpio_get(KB0)); //! toggle the pin
        __asm volatile("nop");
    }
 }

void init_BobSPI() {
    /// @brief used to set up the AD7699 then de-inits the GPIOs so they can be re-initialized to use PIO 
    // First dummy conversion using bit-banging
    gpio_init(Bob_CNV);
    gpio_init(Bob_SCK);
    gpio_init(Bob_MISO);
    gpio_init(Bob_MOSI);
    
    gpio_set_dir(Bob_CNV, GPIO_OUT);
    gpio_set_dir(Bob_SCK, GPIO_OUT);
    gpio_set_dir(Bob_MISO, GPIO_IN);
    gpio_set_dir(Bob_MOSI, GPIO_OUT);

    // Do dummy conversions...
    // First dummy conversion
    gpio_put(Bob_CNV, 0);
    for(int i = 13; i >= 0; i--) { //< MSB First (13 down to 0) (14 bits)
        //! adc_cfg >> i shifts the bits to the right by i places to iterate through the CFG
        gpio_put(Bob_MOSI, (ADC_CFG_DEFAULT >> i) & 0x01); //! ADC_CFG_DEFAULT is the same as adc_cfg[0]
        // printf("bit %d: %d\n", i, ((ADC_CFG_DEFAULT >> i) & 0x01));
        gpio_put(Bob_SCK, 1);
        // sleep_us(1);
        delay_idea(1);
        gpio_put(Bob_SCK, 0);
        // sleep_us(1);
        delay_idea(1);
    }
    gpio_put(Bob_CNV, 1);
    // sleep_us(1);
    delay_idea(1);
    // Second dummy conversion
    gpio_put(Bob_CNV, 0);
    for(int i = 13; i >= 0; i--) {
        gpio_put(Bob_MOSI, (adc_cfg[0] >> i) & 0x01);
        gpio_put(Bob_SCK, 1);
        // sleep_us(1);
        delay_idea(1);
        gpio_put(Bob_SCK, 0);
        // sleep_us(1);
        delay_idea(1);
    }
    gpio_put(Bob_CNV, 1);

    // After dummy conversions, deinitialize the SPI pins so PIO can take over
    gpio_deinit(Bob_CNV); //! do not DeInit CNV because it will be used outside of PIO
    gpio_deinit(Bob_SCK);
    gpio_deinit(Bob_MISO);
    gpio_deinit(Bob_MOSI);
}

void init_leds(void) {
    // Only initialize LED and KB0 pins
    gpio_init_mask((1 << LED_R) | (1 << LED_G) | (1 << LED_B));
    gpio_set_dir_out_masked((1 << LED_R) | (1 << LED_G) | (1 << LED_B));
    gpio_put(LED_B, 1); 
    gpio_put(LED_G, 1); 
    gpio_put(LED_R, 0);  
}

int main()
{
    stdio_init_all();
    init_leds();
    init_BobSPI();
    sleep_ms(1000); //! Delay for stdio
    // Initialize PIO before starting timer
    ad7699_pio_init();
    
    repeating_timer_t timer;
    
    // Use a longer initial delay for setup
    sleep_ms(100);
    
    // Start timer with 2ms period (500Hz)
    /// @note actual interrupt is supposed to run at 500kHz (2us period)
    // add_repeating_timer_ms(2, bob_irq, NULL, &timer);
    add_repeating_timer_us(2, bob_irq, NULL, &timer);
    // add_repeating_timer_us(20, bob_irq, NULL, &timer);
    // add_repeating_timer_us(2000, bob_irq, NULL, &timer);
    // add_repeating_timer_us(4000, bob_irq, NULL, &timer); //! @attention trying a longer delay to see if that is causing problems
    
    while (true) {
        // sleep_ms(1000);
        sleep_ms(100);
        for(int i = 0; i < 8; i++) {
            printf("%d: IN%d: %d = %0.3f\n", 
                adc_flags[i], (i+2)%8, adc_data_store[i], 
                TRANSLATE_TO_FL(adc_data_store[i]));
        }
        puts("\n");
    }
}
