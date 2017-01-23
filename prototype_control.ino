/*------------------------------------------------------------------------*//**
 * \file c_template.c
 * \brief
 *
 *  Original Author: Mitchell Burke
 *
 *  Creation Date: 4 Jan 2015
 *---------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
System level includes
-----------------------------------------------------------------------------*/

#include <LiquidCrystal.h>

/*-----------------------------------------------------------------------------
Project level includes
-----------------------------------------------------------------------------*/
/* None */

/*-----------------------------------------------------------------------------
Local includes
-----------------------------------------------------------------------------*/
/* None */

/*-----------------------------------------------------------------------------
Public data
-----------------------------------------------------------------------------*/
/* None */

/*-----------------------------------------------------------------------------
Private defines
-----------------------------------------------------------------------------*/

/* Pin numbers */
#define PIN_ID_PUSH_BUTTON                          (2)
#define PIN_ID_FLOAT_SWITCH                         (A0)
#define PIN_ID_POD_SWITCH                           (A1)

#define PIN_ID_DISPENSING_PUMP_MOTOR_FORWARD        (13)
#define PIN_ID_DISPENSING_PUMP_MOTOR_REVERSE        (8)

#define PIN_ID_METERING_MOTOR_FORWARD               (12)
#define PIN_ID_METERING_MOTOR_REVERSE               (9)

#define PIN_ID_LCD_RS                               (7)
#define PIN_ID_LCD_ENABLE                           (11)
#define PIN_ID_LCD_D4                               (5)
#define PIN_ID_LCD_D5                               (4)
#define PIN_ID_LCD_D6                               (3)
#define PIN_ID_LCD_D7                               (1)

#define PIN_ID_RECIRCULTING_MIXING_PUMP_RELAY       (6)
#define PN_ID_PELTIER_FAN_RELAY                     (7)
#define PN_ID_LED                                   (10)

/* Debounce values for push buttons and switches */
#define PUSH_BUTTON_DEBOUNCE_MS                     (50)
#define FLOAT_SWITCH_DEBOUNCE_MS                    (100)
#define POD_SWITCH_DEBOUNCE_MS                      (100)

#define DISPENS_TIME_MS                             (5000)

#define TIMER_INVALID                               (0)

/*-----------------------------------------------------------------------------
Private data types
-----------------------------------------------------------------------------*/

/* The states for the state machine */
typedef enum
{
    STATE_READY,
    STATE_NO_POD,
    STATE_LOW_WATER,
    STATE_DISPENSING
} state_t;

/* The events that get passed into the state machine */
typedef enum
{
    EVENT_NONE,
    EVENT_WATER_LEVEL_LOW,
    EVENT_WATER_LEVEL_OKAY,
    EVENT_POD_REMOVED,
    EVENT_POD_REPLACED,
    EVENT_DISPENSE,
    EVENT_TIMEOUT
} event_t;

/* Possible ways to drive the motor */
typedef enum
{
    MOTOR_OFF,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} motor_state_t;

/* All the information assiciated with a push button */
typedef struct
{
    int pin_id; /* The pin number */
    int last_state; /* The last debounced state of the pin */
    long time_last_change; /* The time when the pin changed state - used for debounce only */
    unsigned long debounce_delay; /* How long the pin must be in a state before it is considered stable */
} push_button_t;

/* All the info about a motor */
typedef struct
{
    int forward_pin; /* Forward pin number */
    int reverse_pin; /* Reverse pin number */
} motor_t;

typedef struct
{
    int relay_pin; /* The pin number */
} output_t;

/*-----------------------------------------------------------------------------
Private data - declare static
-----------------------------------------------------------------------------*/

/* state machine current state */
static state_t sm_current_state;

/* Timer info */
static unsigned long timer;

/* device information */
static push_button_t push_button;
static push_button_t float_switch;
static push_button_t pod_switch;

static motor_t dispensing_pump;
static motor_t metering_motor;

static output_t recirculating_mixing_pump;
static output_t peltier_fan_relay;
static output_t led;

/* LCD config */
LiquidCrystal lcd(PIN_ID_LCD_RS, PIN_ID_LCD_ENABLE, PIN_ID_LCD_D4, PIN_ID_LCD_D5, PIN_ID_LCD_D6, PIN_ID_LCD_D7);

/*-----------------------------------------------------------------------------
Private functions Prototypes - declare static
-----------------------------------------------------------------------------*/

static void init_state_machine(void);
static bool run_state_machine(event_t new_event);

static bool ready_state(event_t new_event);
static bool no_pod_state(event_t new_event);
static bool low_water_state(event_t new_event);
static bool dispensing_state(event_t new_event);

static event_t check_for_new_events(void);

static void enter_ready_state(void);
static void enter_low_water_state(void);
static void enter_dispensing_state(void);
static void enter_pod_removed_state(void);

static void set_timer(long timeout_ms);
static bool check_timer_event(void);

static void init_lcd(void);
static void init_push_button(push_button_t *button_info, int off_state, int pin_id, int debounce_time_ms);
static void init_motor_pins(motor_t *motor_info, int forward_pin, int reverse_pin, motor_state_t initial_state);
static void init_output(output_t *output_info, int pin_id, int initial_state);

static bool check_for_push_button_event(push_button_t *pin_info, int new_state);
static int get_push_button_state(push_button_t *pin_info);
static void set_motor_state(motor_t *motor_info, motor_state_t new_state);
static void set_output_state(output_t *output_info, int new_state);

static void display_text(const char *str);
static void handle_error(const char *error_str);

/*-----------------------------------------------------------------------------
Public Function implementations
-----------------------------------------------------------------------------*/

/*************************************************************************//**
 * \brief Call by Arduino to initialise everything
 ****************************************************************************/
void setup()
{
    init_lcd();

    init_push_button(&push_button, LOW, PIN_ID_PUSH_BUTTON, PUSH_BUTTON_DEBOUNCE_MS);
    init_push_button(&float_switch, HIGH, PIN_ID_FLOAT_SWITCH, FLOAT_SWITCH_DEBOUNCE_MS);
    init_push_button(&pod_switch, HIGH, PIN_ID_POD_SWITCH, POD_SWITCH_DEBOUNCE_MS);

    init_motor_pins(&dispensing_pump, PIN_ID_DISPENSING_PUMP_MOTOR_FORWARD, PIN_ID_DISPENSING_PUMP_MOTOR_REVERSE, MOTOR_OFF);
    init_motor_pins(&metering_motor, PIN_ID_METERING_MOTOR_FORWARD, PIN_ID_METERING_MOTOR_REVERSE, MOTOR_OFF);

    init_output(&recirculating_mixing_pump, PIN_ID_RECIRCULTING_MIXING_PUMP_RELAY, LOW);
    init_output(&peltier_fan_relay, PN_ID_PELTIER_FAN_RELAY, LOW);
    init_output(&led, PN_ID_LED, LOW);

    init_state_machine();
}

/*************************************************************************//**
 * \brief Continuously looped until power is cut
 ****************************************************************************/
void loop()
{
    event_t new_event = check_for_new_events();
    run_state_machine(new_event);
}

/*-----------------------------------------------------------------------------
Private Function implementations
-----------------------------------------------------------------------------*/

/*************************************************************************//**
 * \brief Initialises the state machine. Set the variables and puts sets up any
 * peripherals.
 ****************************************************************************/
static void init_state_machine(void)
{
    timer = TIMER_INVALID;
    enter_ready_state();
}

/*************************************************************************//**
 * \brief Runs the state machine with an event.
 *
 * \param new_event - The event to be passed into the state machine
 *
 * \return If the event was was actioned or not
 ****************************************************************************/
static bool run_state_machine(event_t new_event)
{
    bool handled = false;
    switch (sm_current_state)
    {
        case STATE_READY:
            handled = ready_state(new_event);
            break;
        case STATE_NO_POD:
            handled = no_pod_state(new_event);
            break;
        case STATE_LOW_WATER:
            handled = low_water_state(new_event);
            break;
        case STATE_DISPENSING:
            handled = dispensing_state(new_event);
            break;
        default:
            handle_error("SM in wrong state");
            break;
    }
    return handled;
}

/*************************************************************************//**
 * \brief Handles the events when we are in the ready state.
 *
 * \param new_event - The event to be passed into the state machine
 *
 * \return If the event was was actioned or not
 ****************************************************************************/
static bool ready_state(event_t new_event)
{
    bool handled = false;
    switch (new_event)
    {
        case EVENT_WATER_LEVEL_LOW:
            enter_low_water_state();
            handled = true;
            break;
        case EVENT_POD_REMOVED:
            enter_pod_removed_state();
            handled = true;
            break;
        case EVENT_DISPENSE:
            enter_dispensing_state();
            handled = true;
            break;
    }
    return handled;
}

/*************************************************************************//**
 * \brief Handles the events when we are in the no_pod state
 *
 * \param new_event - The event to be passed into the state machine
 *
 * \return If the event was was actioned or not
 ****************************************************************************/
static bool no_pod_state(event_t new_event)
{
    bool handled = false;
    switch (new_event)
    {
        case EVENT_POD_REPLACED:
            /* The pod has been replaced but we just need to check that the water level
             * isn't low. If it is then we need to go to the low water state not the
             * ready state */
            if (get_push_button_state(&float_switch) == LOW)
            {
                enter_low_water_state();
            }
            else
            {
                enter_ready_state();
            }
            handled = true;
            break;
    }
    return handled;
}

/*************************************************************************//**
 * \brief Handles the events when we are in the low water state
 *
 * \param new_event - The event to be passed into the state machine
 *
 * \return If the event was was actioned or not
 ****************************************************************************/
static bool low_water_state(event_t new_event)
{
    bool handled = false;
    switch (new_event)
    {
        case EVENT_WATER_LEVEL_OKAY:
            /* The water is full but the pos may be removed so just just where
             * we should go - pos removed or ready */
            if (get_push_button_state(&pod_switch) == LOW)
            {
                enter_pod_removed_state();
            }
            else
            {
                enter_ready_state();
            }
            handled = true;
            break;
    }
    return handled;
}

/*************************************************************************//**
 * \brief Handles the events when we are in the dispensing state
 *
 * \param new_event - The event to be passed into the state machine
 *
 * \return If the event was was actioned or not
 ****************************************************************************/
static bool dispensing_state(event_t new_event)
{
    bool handled = false;
    switch (new_event)
    {
        case EVENT_TIMEOUT:
            /* We are done dispensing but may need to go to the pod removed
             * or the low water state instead of the ready state */
            if (get_push_button_state(&pod_switch) == LOW)
            {
                enter_pod_removed_state();
            }
            else if (get_push_button_state(&float_switch) == LOW)
            {
                enter_low_water_state();
            }
            else
            {
                enter_ready_state();
            }
            handled = true;
            break;
    }
    return handled;
}

/*************************************************************************//**
 * \brief Checks the system to see if an event happened
 *
 * \return The event that happened
 ****************************************************************************/
static event_t check_for_new_events(void)
{
    event_t event = EVENT_NONE;
    if (check_for_push_button_event(&push_button, HIGH))
    {
        event = EVENT_DISPENSE;
    }
    else if (check_for_push_button_event(&float_switch, HIGH))
    {
        event = EVENT_WATER_LEVEL_OKAY;
    }
    else if (check_for_push_button_event(&float_switch, LOW))
    {
        event = EVENT_WATER_LEVEL_LOW;
    }
    else if (check_for_push_button_event(&pod_switch, HIGH))
    {
        event = EVENT_POD_REPLACED;
    }
    else if (check_for_push_button_event(&pod_switch, LOW))
    {
        event = EVENT_POD_REMOVED;
    }
    else if (check_timer_event())
    {
        event = EVENT_TIMEOUT;
    }

    return event;
}

/*************************************************************************//**
 * \brief Puts all the peripherals into the ready state.
 ****************************************************************************/
static void enter_ready_state(void)
{
    display_text("Ready");

    /* Turn the motors off */
    set_motor_state(&dispensing_pump, MOTOR_OFF);
    set_motor_state(&metering_motor, MOTOR_OFF);

    /* Turn on recirculating pump */
    set_output_state(&recirculating_mixing_pump, HIGH);

    /* Turn on the peltier and fan */
    set_output_state(&peltier_fan_relay, HIGH);

    /* Turn off the LED */
    set_output_state(&led, LOW);

    sm_current_state = STATE_READY;
}

/*************************************************************************//**
 * \brief Puts all the peripherals into the low water state
 ****************************************************************************/
static void enter_low_water_state(void)
{
    display_text("Low water");
    //turn off recurculating pump
    set_output_state(&recirculating_mixing_pump, LOW);

    sm_current_state = STATE_LOW_WATER;
}

/*************************************************************************//**
 * \brief Puts all the peripherals into the dispensing state
 ****************************************************************************/
static void enter_dispensing_state(void)
{
    display_text("Dispensing");
    //turn off recirculating pump
    set_output_state(&recirculating_mixing_pump, LOW);

    //turn on dispensing pump
    set_motor_state(&dispensing_pump, MOTOR_FORWARD);
    set_motor_state(&metering_motor, MOTOR_FORWARD);

    //start timer
    set_timer(DISPENS_TIME_MS);

    sm_current_state = STATE_DISPENSING;
}

/*************************************************************************//**
 * \brief Puts all the peripherals into the pod removed state
 ****************************************************************************/
static void enter_pod_removed_state(void)
{
    display_text("Replace Pod");
    sm_current_state = STATE_NO_POD;
}

/*************************************************************************//**
 * \brief Starts a timer running
 *
 * \param in_time_ms - The timer will go off in this amount of time
 ****************************************************************************/
static void set_timer(long timeout_ms)
{
    if (timer == TIMER_INVALID)
    {
        timer = millis() + timeout_ms;

        /* This is just incase the timer wraps around - it cant be zero
         * because thats invalid */
        if (timer == TIMER_INVALID)
        {
            timer = 1;
        }
    }
    else
    {
        handle_error("Timer alreadyset");
    }
}

/*************************************************************************//**
 * \brief Checks to see if the timer has gone off
 *
 * \return If the timer has gone off
 ****************************************************************************/
static bool check_timer_event(void)
{
    bool time_up = false;

    if (timer != TIMER_INVALID)
    {
        if (millis() >= timer)
        {
            /* Just put in a check to make sure the timer hasn't wrapped around
             * If the time is greater than 1 day then the value has wrapped around */
            if (millis() - timer > (24*60*60*1000))
            {
                /* Do nothing wait for the time to catch up */
            }
            else
            {
                timer = TIMER_INVALID;
                time_up = true;
            }
        }
    }

    return time_up;
}

/*************************************************************************//**
 * \brief Initialises the LCD
 ****************************************************************************/
static void init_lcd(void)
{
    pinMode(PIN_ID_LCD_RS, OUTPUT);
    pinMode(PIN_ID_LCD_ENABLE, OUTPUT);
    pinMode(PIN_ID_LCD_D4, OUTPUT);
    pinMode(PIN_ID_LCD_D5, OUTPUT);
    pinMode(PIN_ID_LCD_D6, OUTPUT);
    pinMode(PIN_ID_LCD_D7, OUTPUT);

    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    lcd.noBlink();
    lcd.clear();
}

/*************************************************************************//**
 * \brief Initialises a push button
 *
 * \param button_info - Pointer to the information about the peripherals to be initialised
 * \param off_state - The Expected state (LOW/HIGH) when the system is in the
 *                  ready state. This is just an initial value to stop events
 *                  being generated when the system boots.
 * \param pin_id - The pin number.
 * \param debounce_time_ms - Time before the pin is considered settled in milliseconds.
 ****************************************************************************/
static void init_push_button(push_button_t *button_info, int off_state, int pin_id, int debounce_time_ms)
{
    pinMode(pin_id, INPUT);
    button_info->last_state = off_state;
    button_info->pin_id = pin_id;
    button_info->time_last_change = 0;
    button_info->debounce_delay = debounce_time_ms;
}

/*************************************************************************//**
 * \brief Initialises a motor.
 *
 * \param motor_info - Pointer to the information about the peripherals to be initialised
 * \param forward_pin - The forward pin id.
 * \param reverse_pin - The reverse pin id
 * \param initial_state - The initial state that you want the motor to be in.
 *                          (MOTOR_OFF/MOTOR_FORWARD/MOTOR_REVERSE)
 ****************************************************************************/
static void init_motor_pins(motor_t *motor_info, int forward_pin, int reverse_pin, motor_state_t initial_state)
{
    pinMode(forward_pin, OUTPUT);
    pinMode(reverse_pin, OUTPUT);
    motor_info->forward_pin = forward_pin;
    motor_info->reverse_pin = reverse_pin;

    set_motor_state(motor_info, initial_state);
}

/*************************************************************************//**
 * \brief Initialise a relay
 *
 * \param output_info - Pointer to information about the peripherals to be initialised
 * \param pin_id - Relay pin id
 * \param initial_state - Initial state the relay should be in (HIGH/LOW)
 ****************************************************************************/
static void init_output(output_t *output_info, int pin_id, int initial_state)
{
    pinMode(pin_id, OUTPUT);
    output_info->relay_pin = pin_id;

    digitalWrite(output_info->relay_pin, initial_state);
}

/*************************************************************************//**
 * \brief This is the push button debounce function. It checks to see if the
 * pin has changed into the new_state. Eg has the pin state just changed to low
 *
 * \param pin_info - Pointer to the push button information.
 * \param new_state - The state we are checking to see if the push button has
 *              just gone into (LOW/HIGH)
 *
 * \return If the push button has changes into that state
 ****************************************************************************/
static bool check_for_push_button_event(push_button_t *pin_info, int new_state)
{
    bool pin_went_high = false;
    int reading = digitalRead(pin_info->pin_id);
    if (reading != pin_info->last_state)
    {
        if (pin_info->time_last_change == 0)
        {
            pin_info->time_last_change = millis();
        }
        else if ((millis() - pin_info->time_last_change) > push_button.debounce_delay)
        {
            pin_info->last_state = reading;
            pin_info->time_last_change = 0;
            if (reading == new_state)
            {
                pin_went_high = true;
            }
        }
    }
    return pin_went_high;
}

/*************************************************************************//**
 * \brief Checks the current push button state
 *
 * \param pin_info - Pointer to the push button information.
 *
 * \return The current debounced state (LOW/HIGH)
 ****************************************************************************/
static int get_push_button_state(push_button_t *pin_info)
{
    return pin_info->last_state;
}

/*************************************************************************//**
 * \brief
 *
 * \param motor_info - Pointer to the motor button information.
 * \param new_state - The desired motor state (MOTOR_OFF/MOTOR_FORWARD/MOTOR_REVERSE)
 ****************************************************************************/
static void set_motor_state(motor_t *motor_info, motor_state_t new_state)
{
    switch (new_state)
    {
        case MOTOR_OFF:
            digitalWrite(motor_info->forward_pin, HIGH);
            digitalWrite(motor_info->reverse_pin, HIGH);
            break;
        case MOTOR_FORWARD:
            digitalWrite(motor_info->forward_pin, HIGH);
            digitalWrite(motor_info->reverse_pin, LOW);
            break;
        case MOTOR_REVERSE:
            digitalWrite(motor_info->forward_pin, LOW);
            digitalWrite(motor_info->reverse_pin, HIGH);
            break;
        default:
            handle_error("bad Motor state");
            break;
    }
}

/*************************************************************************//**
 * \brief Change the relay output to the desired value
 *
 * \param output_info - Pointer to the relay information
 * \param new_state - The desired relay output (LOW/HIGH)
 ****************************************************************************/
static void set_output_state(output_t *output_info, int new_state)
{
    digitalWrite(output_info->relay_pin, new_state);
}

/*************************************************************************//**
 * \brief Writes some text to the LCD
 *
 * \param output_info - The string to Write eg: display_text("Fuck you")
 ****************************************************************************/
/* Print function to help with printing */
static void display_text(const char *str)
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(str);
}

/*************************************************************************//**
 * \brief This is an error handler. If called it prints a message and halts
 * the system.
 *
 * \param error_str - The string to be printed. Error should be printed on the
 *          line above the message.
 ****************************************************************************/
static void handle_error(const char *error_str)
{
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Error");
    lcd.setCursor(0,1);
    lcd.print(error_str);
    while (1)
    {
        ;
    }
}

/*-----------------------------------------------------------------------------
End of file
-----------------------------------------------------------------------------*/
