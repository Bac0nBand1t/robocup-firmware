#include <stdbool.h>

#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stddef.h>
#include <stdlib.h>
#include <util/delay.h>

#include "HAL_attiny167.h"

#include "kicker_config.h"
#include "kicker_commands.h"
#include "pins.h"

// kicker parameters
#define MAX_KICK_STRENGTH 15.0f //2^4 - 1
#define MIN_EFFECTIVE_KICK_FET_EN_TIME 0.8f
#define MAX_EFFECTIVE_KICK_FET_EN_TIME 10.0f

#define KICK_TIME_SLOPE \
    (MAX_EFFECTIVE_KICK_FET_EN_TIME - MIN_EFFECTIVE_KICK_FET_EN_TIME)

// How much time to give for the LT to stop charging the caps
#define STOP_CHARGING_SAFETY_MARGIN_MS 5
// How much time to give the FET to stop current flow
#define STOP_FLOW_SAFETY_MARGIN_MS 5
// Time after the kick to allow the caps to charge back up somewhat
#define CHARGE_TIME_MS 2000

// number of steps of resolution we want per millisecond
#define TIMER_PER_MS 4

// calculate our TIMING_CONSTANT (timer cmp val) from the desired resolution
#define CLK_FREQ 8000000  // after removing default CLKDIV8 prescale
#define TIMER_PRESCALE \
   8  // set by TCCR0B |= _BV(CS01) which also starts the timer
#define MS_PER_SECOND 1000

#define MAX_TIMER_FREQ (CLK_FREQ / TIMER_PRESCALE)
#define DESIRED_TIMER_FREQ (TIMER_PER_MS * MS_PER_SECOND)

#define TIMING_CONSTANT ((MAX_TIMER_FREQ / DESIRED_TIMER_FREQ) - 1)

// get different ball reading for X * 10 us before switching
#define BALL_SENSE_MAX_SAMPLES 5

// Corresponds to the values of the kick_type_is_kick
#define IS_KICK true
#define IS_CHIP false

// Used to time kick and chip durations, -1 indicates inactive state
volatile struct {
    volatile int32_t stop_charge_phase; // Allow time for main loop to stop charge
    volatile int32_t flow_phase;        // Let current flow during this phase
    volatile int32_t stop_flow_phase;   // Allow time for current to stop flowing
    volatile int32_t charge_phase;      // Allow time for caps to charge back up
} time = {-1, -1, -1, -1};

// Latest command from mtrain
volatile struct {
    volatile bool kick_type_is_kick; // chip or kick?
    volatile bool kick_immediate; // Kick immediately?
    volatile bool kick_on_breakbeam; // Kick when breakbeam triggers?
    volatile bool commanded_charge; // Commanded to charge the caps?
    volatile uint8_t kick_power; // Commanded power to kick at
} command = {false, false, false, false, 0};

// Current kick command
volatile bool current_kick_type_is_kick = true;

// Global vars so we don't have to read pins in interrupts etc
volatile uint8_t current_voltage = 0;
volatile bool ball_sensed = false;

// Global state variables
volatile bool in_debug_mode = false;
volatile bool charge_allowed = true; // Don't charge during kick

void init();

/*
 * Checks and returns if we're in the middle of a kick
 */
bool is_kicking() {
    return time.stop_charge_phase >= 0 ||
           time.flow_phase >= 0 ||
           time.stop_flow_phase >= 0 ||
           time.charge_phase >= 0;
}

/**
 * start the kick FSM for desired strength. If the FSM is already running,
 * the call will be ignored.
 */
void kick(uint8_t strength, bool is_kick) {
    // check if the kick FSM is running
    if (is_kicking()) return;

    // initialize the countdowns for pre and post kick
    time.stop_charge_phase = (STOP_CHARGING_SAFETY_MARGIN_MS * TIMER_PER_MS);

    // compute time the solenoid FET is turned on, in milliseconds, based on
    // min and max effective FET enabled times
    float strength_ratio = (strength / MAX_KICK_STRENGTH);
    float time_cnt_flt_ms =
        KICK_TIME_SLOPE * strength_ratio + MIN_EFFECTIVE_KICK_FET_EN_TIME;
    float time_cnt_flt = time_cnt_flt_ms * TIMER_PER_MS;
    time.flow_phase = (int)(time_cnt_flt + 0.5f);  // round

    time.stop_flow_phase   = (STOP_FLOW_SAFETY_MARGIN_MS * TIMER_PER_MS);

    // force to int32_t, default word size too small
    time.charge_phase = ((int32_t)CHARGE_TIME_MS) * TIMER_PER_MS;

    // Set kick type after we have commited to the kick
    // such that it doesn't change halfway through the kick
    current_kick_type_is_kick = is_kick;

    // start timer to enable the kick FSM processing interrupt
    TCCR0B |= _BV(CS01); // No prescale
}

void handle_debug_mode() {
    // Used to keep track of current button state
    static bool kick_db_down = true;
    static bool chip_db_down = true;
    static bool charge_db_down = true;

    if (in_debug_mode) {
        // Check if the buttons are pressed
        bool kick_db_pressed = !(HAL_IsSet(DB_KICK_PIN));
        bool chip_db_pressed = !(HAL_IsSet(DB_CHIP_PIN));
        bool charge_db_pressed = !(HAL_IsSet(DB_CHG_PIN));

        // Simple rising edge triggers
        if (!kick_db_down && kick_db_pressed) 
            kick(255, IS_KICK);

        if (!chip_db_down && chip_db_pressed)
            kick(255, IS_CHIP);

        // If we should be charging
        if (!charge_db_down && charge_db_pressed)
            command.commanded_charge = !command.commanded_charge;

        kick_db_down = kick_db_pressed;
        chip_db_down = chip_db_pressed;
        charge_db_down = charge_db_pressed;
    }
}

uint8_t get_voltage() {
    // Start conversation by writing to start bit
    ADCSRA |= _BV(ADSC);

    // Wait for ADSC bit to clear
    while (ADCSRA & _BV(ADSC));

    // ADHC will range from 0 to 255 corresponding to 0 through VCC
    return ADCH;
}

void try_read_voltage() {
    static uint32_t time = 0;

    // Don't run the adc every loop
    // 1000 * 10 us = 10 ms
    if (time % 1000 == 0) {
        current_voltage = get_voltage();
    }
    time++;
}

void update_ball_sense() {
    static uint32_t ball_sense_change_count = 0;

    // Filter ball value
    // Want X amount in a row to be the same
    bool new_reading = HAL_IsSet(BALL_SENSE_RX);

    // If we sensed the ball, but don't think it's triggered
    // or we didn't sense the ball, and think it's triggered
    // AKA our estimate is wrong
    if (ball_sensed ^ new_reading)
        ball_sense_change_count++;
    else // else correct reading
        ball_sense_change_count = 0;

    // We got a wrong reading X times in a row
    // so we should swap
    if (ball_sense_change_count > BALL_SENSE_MAX_SAMPLES) {
        ball_sense_change_count = 0;

        ball_sensed = !ball_sensed;
    }
}

void charge_caps() {
    // if we dropped below acceptable voltage, then this will catch it
    // note: these aren't true voltages, just ADC output, but it matches
    // fairly close
    
    // Stop charging if we are at the voltage target
    if (current_voltage > 244 ||
        !charge_allowed ||
        !command.commanded_charge) {

        HAL_ClearPin(LT_CHARGE);

    // Charge if we are too low
    } else if (current_voltage < 239 &&
               charge_allowed &&
               command.commanded_charge) {

        HAL_SetPin(LT_CHARGE);
    }
}

void fill_spi_return() {
    uint8_t ret_byte = 0x00;

    if (ball_sensed)
        ret_byte |= BREAKBEAM_TRIPPED;

    ret_byte |= VOLTAGE_MASK & (current_voltage >> 1);

    SPDR = ret_byte;
}

void main() {
    init();

    while (true) {

        handle_debug_mode();

        try_read_voltage();

        update_ball_sense();

        charge_caps();

        // Kick on give command
        if ((command.kick_on_breakbeam && ball_sensed) ||
            command.kick_immediate) {

            // pow
            kick(command.kick_power, command.kick_type_is_kick);
            command.kick_immediate = false;
            command.kick_on_breakbeam = false;
        }

        fill_spi_return();

        _delay_us(10);
    }
}

/*
 * SPI Interrupt. Triggers when we have a new byte available, it'll be
 * stored in SPDR. Writing a response also occurs using the SPDR register.
 * 
 * Receive the command and set the global variables accordingly
 */
ISR(SPI_STC_vect) {
    // Don't take commands in debug mode
    if (in_debug_mode)
        return;
    
    uint8_t recv_data = SPDR;
    
    // Fill our globals with the commands
    command.kick_type_is_kick = (recv_data & TYPE_FIELD) == TYPE_KICK;
    command.commanded_charge  = recv_data & CHARGE_ALLOWED;
    command.kick_power        = recv_data & KICK_POWER_MASK;

    // If chip, force max power
    if (command.kick_type_is_kick == IS_CHIP) {
        command.kick_power = 0xF;
    }

    // If we get a cancel kick command
    // Stop the kicks
    if ((recv_data & CANCEL_KICK) == CANCEL_KICK) {
        command.kick_immediate = false;
        command.kick_on_breakbeam = false;

    // Set the correct kick action
    } else if (recv_data & KICK_IMMEDIATE) {
        command.kick_immediate    = true;
        command.kick_on_breakbeam = false;
    } else if (recv_data & KICK_ON_BREAKBEAM) {
        command.kick_immediate    = false;
        command.kick_on_breakbeam = true;
    }
}

/**
 * Timer interrupt for chipping/kicking - called every millisecond by timer
 *
 * ISR for TIMER 0
 *
 * Pre and post cool downs add time between kicking and charging
 *
 * Charging while kicking is destructive to the charging circuitry
 * If no outstanding coutners are running from the timer, the pre, active, post,
 *and cooldown
 * states are all finished. We disable the timer to avoid unnecessary ISR
 *invocations when
 * there's nothing to do. The kick function will reinstate the timer.
 *
 * TCCR0B:
 * CS00 bit stays at zero
 * When CS01 is also zero, the clk is diabled
 * When CS01 is one, the clk is prescaled by 8
 * (When CS00 is one, and CS01 is 0, no prescale. We don't use this)
 */
ISR(TIMER0_COMPA_vect) {
    if (time.stop_charge_phase >= 0) {
        /**
         * PRE KICKING STATE
         * stop charging
         * wait between stopping charging and kicking for safety
         */
        // disable charging
        charge_allowed = false;

        time.stop_charge_phase--;
    } else if (time.flow_phase >= 0) {
        /**
         * KICKING STATE
         * assert the kick pin, enabling the kick FET
         * wait for kick interval to end
         */

        // todo
        if (current_kick_type_is_kick == IS_KICK) {
            HAL_SetPin(KICK_PIN);
        } else {
            HAL_SetPin(CHIP_PIN);
        }

        time.flow_phase--;
    } else if (time.stop_flow_phase >= 0) {
        /**
         * POST KICKING STATE
         * deassert the kick pin, disabling the kick FET
         * wait between stopping the FET and reenabling charging in the next
         * state
         */

        // todo
        if (current_kick_type_is_kick == IS_KICK) {
            HAL_ClearPin(KICK_PIN);
        } else {
            HAL_ClearPin(CHIP_PIN);
        }

        time.stop_flow_phase--;
    } else if (time.charge_phase >= 0) {
        /**
         * POST KICK COOLDOWN
         * enable charging
         * don't allow kicking during the cooldown
         */

        // reenable charging
        charge_allowed = true;

        time.charge_phase--;
    } else {
        /**
         * IDLE/NOT RUNNING
         * stop timer
         */

        // stop prescaled timer
        TCCR0B &= ~_BV(CS01); // todo
    }
}

void init() {
    // Disable pullups globally
    MCUCR |= _BV(PUD);
    
    // disable interrupts
    cli();

    // disable watchdog
    wdt_reset();
    MCUSR &= !_BV(WDRF);
    WDTCR |= (_BV(WDCE) | _BV(WDE));
    WDTCR = 0x00; // WDTON WDE WDIE have to be disabled
    
    // change our clock speed from 1 Mhz to 8 Mhz (there's a default CLKDIV8)
    // 1. write the Clock Prescaler Change Enable (CLKPCE) bit to one and all
    //    other bits in CLKPR to zero.
    // 2. within four cycles, write the desired value to CLKPS while writing a
    //    zero to CLKPCE.
    CLKPR = _BV(CLKPCE);
    CLKPR = 0;  // corresponds to CLKDIV1 prescale, also keeps CLKPCE low

    /**
     * Input button initialization
     */

    HAL_SetInputPin(DB_SWITCH);
    HAL_SetInputPin(DB_CHG_PIN);
    HAL_SetInputPin(DB_KICK_PIN);
    HAL_SetInputPin(DB_CHIP_PIN);


    /**
     * SPI initialization
     */

    // MISO as output
    // CS and MOSI as input
    HAL_SetInputPin(KICK_MOSI_PIN);
    HAL_SetOutputPin(KICK_MISO_PIN); // must be configured manually
    HAL_SetInputPin(N_KICK_CS_PIN);
    HAL_SetInputPin(KICK_MOSI_PIN);
    // All other autoconfigured on spi enable

    /**
     * LT3751 initialization
     */
    // Default values for charge
    HAL_ClearPin(LT_CHARGE);

    // Charge command output
    HAL_SetOutputPin(LT_CHARGE);

    HAL_ClearPin(KICK_PIN);
    HAL_ClearPin(CHIP_PIN);

    HAL_SetOutputPin(KICK_PIN);
    HAL_SetOutputPin(CHIP_PIN);

    /**
     * Ball sense initialization
     */
    // Default ball sense on startup
    HAL_ClearPin(BALL_SENSE_TX);

    // tx and led as output
    // RX as intput
    HAL_SetInputPin(BALL_SENSE_RX);
    HAL_SetOutputPin(BALL_SENSE_TX);

    // Enable the LED and TX until first loop
    // This is because you cannot go from {input, tristate} -> {output, high}
    // in a single step
    HAL_SetPin(BALL_SENSE_TX);
    
    /**
     * Enable SPI slave
     */
    // Assume default spi mode
    // Allow interrupts, enable, and force slave SPI device
    SPCR &= ~_BV(MSTR);
    SPCR |= _BV(SPE) | _BV(SPIE);

    ///////////////////////////////////////////////////////////////////////////
    //  TIMER INITIALIZATION
    //
    //  The timer works by interrupt callback. The timer is based off of an
    //  accumulator register that is incremented per clock tick. When the
    //  accumulator register reaches the value in the target register,
    //  the interrupt fires.
    //
    //  Initialization
    //  1) The interrupt enable bit is set in TIMSK0
    //  2) Clear the bit indicating timer matched the target/compare register
    //  3) Set the value of the target/compare register
    //
    //  Callback
    //  ISR(TIMER0_COMPA_vect)
    //
    //  Start/Global
    //  kick()
    //
    //  initialize timer
    TIMSK0 |= _BV(OCIE0A);    // Interrupt on TIMER 0
    TCCR0A |= _BV(WGM01); // CTC
    OCR0A = TIMING_CONSTANT;  // OCR0A is max val of timer before reset
    ///////////////////////////////////////////////////////////////////////////


    /**
     * ADC Initialization
     */
    // Allow us to just read a single register to get the analog output
    // instead of having to read 2 registers
    // We don't care about the full 10 bit width, only the top 8
    // Setup voltage monitor as input to adc
    ADMUX |= _BV(ADLAR) | 0x00;//_BV(V_MONITOR_PIN.pin);

    //  Ensure ADC isn't off
    PRR &= ~_BV(PRADC);

    // Enable adc
    ADCSRA |= _BV(ADEN);

    /**
     * Button logic
     */
    // latch debug state
    in_debug_mode = !HAL_IsSet(DB_SWITCH);

    // enable global interrupts
    sei();
}
