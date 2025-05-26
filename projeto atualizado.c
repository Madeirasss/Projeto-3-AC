#include <reg51.h>

/* ===================== HARDWARE PIN DEFINITIONS ===================== */
sbit STATUS_LED_GREEN_PIN     = P1^0;  // Green Status LED (Active Low: 0=ON)
sbit STATUS_LED_RED_PIN       = P1^1;  // Red Status LED (Active Low)
sbit STATUS_LED_YELLOW_PIN    = P1^2;  // Yellow Status LED (Active Low)
sbit SERVO_CTRL_PIN           = P1^3;  // Servo motor control output
sbit VEHICLE_DETECT_SENSOR_PIN = P1^4; // Optical sensor (1=No vehicle, 0=Vehicle present)

/* ===================== SYSTEM PARAMETERS ===================== */
#define MAX_PARKING_CAPACITY            8
#define GATE_AUTO_CLOSE_DELAY_SEC       10

/* ===================== TIMER SETTINGS ===================== */
// Timer0 - General purpose timing (e.g., yellow LED blink, gate open duration)
// Using a 12MHz crystal
#define GATE_OPEN_MIN_DURATION_TICKS    (GATE_AUTO_CLOSE_DELAY_SEC * CLOCK_TICKS_PER_SECOND)
#define YELLOW_LED_TOGGLE_TICKS         CLOCK_TICKS_PER_SECOND  // Blink every 1 second
#define T0_RELOAD_HIGH_BYTE             0x3C  // For a 50ms interrupt interval
#define T0_RELOAD_LOW_BYTE              0xB0
#define CLOCK_TICKS_PER_SECOND          20    // Based on 50ms Timer0 interval

// Timer1 - Servo Motor PWM Generation (target 20ms period)
#define SERVO_PWM_PERIOD_US             20000
#define SERVO_PULSE_MIN_US              1000  // Gate closed position (~1ms)
#define SERVO_PULSE_MAX_US              2000  // Gate open position (~2ms)

// Macro to calculate timer reload value from microseconds
#define CALCULATE_TIMER_RELOAD_US(microseconds)  (65536 - (microseconds))

/* ===================== GLOBAL STATE FLAGS & COUNTERS ===================== */
volatile unsigned char g_availableParkingSlots = MAX_PARKING_CAPACITY;
volatile bit g_isGateSequenceActive = 0;     // Flag indicating gate operation is in progress
volatile unsigned int g_gateOpenTimerTicks = 0;
volatile unsigned int g_yellowLedBlinkTimerTicks = 0;
volatile bit g_yellowLedState = 1;          // Tracks yellow LED pin state (1=OFF, 0=ON due to active low)

// Servo control related globals
volatile bit g_servoPwmRunning = 0;
volatile unsigned int g_currentServoPulseUs = SERVO_PULSE_MIN_US;
volatile bit g_isServoSignalHigh = 0;

/* ===================== FUNCTION DECLARATIONS ===================== */
void system_init(void);
void setupTimer1ForServo(void);
void displaySlotCountOnSevenSegment(unsigned char count);
void readParkingBaySensors(void);
void updateParkingStatusIndicators(void);

/* ===================== INITIALIZATION ROUTINES ===================== */
void system_init(void) {
    // Initialize Ports
    P0 = 0xFF; // Port 0 as input for parking bay sensors
    P1 = 0xFF; // Initialize P1: LEDs OFF (active low), Servo LOW, Vehicle Sensor as input
               // Specifically:
    STATUS_LED_GREEN_PIN  = 1; // Off
    STATUS_LED_RED_PIN    = 1; // Off
    STATUS_LED_YELLOW_PIN = 1; // Off
    SERVO_CTRL_PIN        = 0; // Servo signal initially low
    VEHICLE_DETECT_SENSOR_PIN = 1; // Set as input

    P2 = 0x00; // Initialize 7-segment display (e.g., show 0 or initial count)

    // Timer 0 Configuration (Mode 1, 16-bit timer)
    TMOD = (TMOD & 0xF0) | 0x01; // Set Timer 0 to Mode 1, leave Timer 1 settings
    TH0 = T0_RELOAD_HIGH_BYTE;
    TL0 = T0_RELOAD_LOW_BYTE;
    ET0 = 1; // Enable Timer 0 interrupt
    TR0 = 0; // Timer 0 initially stopped, started by gate operations

    // External Interrupts Configuration
    IT0 = 1; // External interrupt 0 (Entry) on falling edge
    IT1 = 1; // External interrupt 1 (Exit) on falling edge
    EX0 = 1; // Enable External interrupt 0
    EX1 = 1; // Enable External interrupt 1

    // Global interrupt enable is done in main after all init
}

void setupTimer1ForServo(void) {
    // Timer 1 Configuration (Mode 1, 16-bit timer for Servo PWM)
    TMOD = (TMOD & 0x0F) | 0x10; // Set Timer 1 to Mode 1, leave Timer 0 settings
    
    g_isServoSignalHigh = 0;
    SERVO_CTRL_PIN = 0;
    g_servoPwmRunning = 1;  // Enable PWM by default

    // Pre-load Timer1 for the low part of the initial PWM cycle (gate closed)
    TH1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(SERVO_PWM_PERIOD_US - g_currentServoPulseUs) >> 8);
    TL1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(SERVO_PWM_PERIOD_US - g_currentServoPulseUs) & 0xFF);
    
    ET1 = 1; // Enable Timer 1 interrupt
    TR1 = 1; // Start Timer 1
}

/* ===================== DISPLAY MANAGEMENT ===================== */
void displaySlotCountOnSevenSegment(unsigned char countToShow) {
    if (countToShow > MAX_PARKING_CAPACITY) {
        countToShow = MAX_PARKING_CAPACITY; // Cap at max
    }
    // Assuming P2 directly drives a 7-segment display or a BCD to 7-seg decoder
    // If direct drive, this should be a lookup table.
    // For simplicity, directly outputting the number if a decoder is assumed.
    P2 = countToShow;
}

/* ===================== SENSOR HANDLING ===================== */
void readParkingBaySensors(void) {
    unsigned char sensorIndex;
    unsigned char activeSensorMask = 0x01;
    unsigned char freeSlotsFound = 0;
    unsigned char p0_data = P0; // Read Port0 once to avoid inconsistencies

    for (sensorIndex = 0; sensorIndex < MAX_PARKING_CAPACITY; sensorIndex++) {
        // Assuming 1 on sensor input means slot is free, 0 means occupied.
        // P0^0 to P0^7 are used for the 8 slots.
        if (p0_data & activeSensorMask) {
            freeSlotsFound++;
        }
        activeSensorMask <<= 1; // Move to next sensor bit
    }
    
    g_availableParkingSlots = freeSlotsFound;
}

void updateParkingStatusIndicators(void) {
    if (g_availableParkingSlots == 0) {
        // Parking full
        STATUS_LED_RED_PIN = 0;   // Red ON
        STATUS_LED_GREEN_PIN = 1; // Green OFF
    } else {
        // Spaces available
        STATUS_LED_RED_PIN = 1;   // Red OFF
        STATUS_LED_GREEN_PIN = 0; // Green ON
    }
}

/* ===================== INTERRUPT SERVICE ROUTINES (ISRs) ===================== */

// Entry Gate Request (INT0)
void entryGateISR(void) interrupt 0 {
    if (!g_isGateSequenceActive && g_availableParkingSlots > 0) {
        g_isGateSequenceActive = 1;
        
        g_currentServoPulseUs = SERVO_PULSE_MAX_US; // Target open position
        g_servoPwmRunning = 1;
        
        // Safely reconfigure Timer1 for new pulse width if not about to overflow
        if (!TF1) { 
            TR1 = 0; // Stop timer for reconfiguration
            g_isServoSignalHigh = 0; // Ensure we start with LOW part next if mid-cycle
            SERVO_CTRL_PIN = 0;
            // Load for LOW duration first, next Timer1 ISR will set it HIGH
            TH1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(SERVO_PWM_PERIOD_US - g_currentServoPulseUs) >> 8);
            TL1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(SERVO_PWM_PERIOD_US - g_currentServoPulseUs) & 0xFF);
            TR1 = 1; // Restart Timer1
        }
        
        g_yellowLedState = 0; // Turn Yellow LED ON (Active Low)
        STATUS_LED_YELLOW_PIN = g_yellowLedState; 
        g_gateOpenTimerTicks = 0;
        g_yellowLedBlinkTimerTicks = 0;
        TR0 = 1; // Start Timer0 for gate timing and LED blinking
    }
}

// Exit Gate Request (INT1)
void exitGateISR(void) interrupt 2 {
    if (!g_isGateSequenceActive) {
        g_isGateSequenceActive = 1;
        
        g_currentServoPulseUs = SERVO_PULSE_MAX_US; // Target open position
        g_servoPwmRunning = 1;

        if (!TF1) {
            TR1 = 0;
            g_isServoSignalHigh = 0;
            SERVO_CTRL_PIN = 0;
            TH1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(SERVO_PWM_PERIOD_US - g_currentServoPulseUs) >> 8);
            TL1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(SERVO_PWM_PERIOD_US - g_currentServoPulseUs) & 0xFF);
            TR1 = 1;
        }
        
        g_yellowLedState = 0; // Turn Yellow LED ON
        STATUS_LED_YELLOW_PIN = g_yellowLedState;
        g_gateOpenTimerTicks = 0;
        g_yellowLedBlinkTimerTicks = 0;
        TR0 = 1; // Start Timer0
    }
}

// Timer0 ISR: System ticks for gate timing and yellow LED blinking
void timer0_SystemTickISR(void) interrupt 1 {
    // Reload Timer0 for next 50ms interval
    TH0 = T0_RELOAD_HIGH_BYTE;
    TL0 = T0_RELOAD_LOW_BYTE;

    if (g_isGateSequenceActive) {
        // Yellow LED blinking logic
        g_yellowLedBlinkTimerTicks++;
        if (g_yellowLedBlinkTimerTicks >= YELLOW_LED_TOGGLE_TICKS) {
            g_yellowLedBlinkTimerTicks = 0;
            g_yellowLedState = !g_yellowLedState; // Toggle state
            STATUS_LED_YELLOW_PIN = g_yellowLedState;   // Update LED (active low handled by state)
        }

        // Gate auto-close logic
        g_gateOpenTimerTicks++;
        if (g_gateOpenTimerTicks >= GATE_OPEN_MIN_DURATION_TICKS) {
            // Check if vehicle has passed (sensor input is 1 if no vehicle)
            if (VEHICLE_DETECT_SENSOR_PIN == 1) { 
                g_currentServoPulseUs = SERVO_PULSE_MIN_US; // Target closed position
                // Timer1 will automatically adjust to this new pulse width in its ISR
                
                g_yellowLedState = 1; // Turn Yellow LED OFF
                STATUS_LED_YELLOW_PIN = g_yellowLedState;
                g_isGateSequenceActive = 0; // End gate sequence
                TR0 = 0; // Stop Timer0 as it's no longer needed
            }
            // If vehicle still present, gate remains open, timer continues
        }
    }
}

// Timer1 ISR: Servo PWM generation
void timer1_ServoPwmISR(void) interrupt 3 {
    if (!g_servoPwmRunning) {
        TR1 = 0; // Stop Timer1 if PWM is disabled
        SERVO_CTRL_PIN = 0;
        g_isServoSignalHigh = 0;
        return;
    }

    if (g_isServoSignalHigh) {
        // Currently HIGH, transition to LOW
        SERVO_CTRL_PIN = 0;
        g_isServoSignalHigh = 0;
        // Set timer for the LOW duration of the PWM cycle
        TH1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(SERVO_PWM_PERIOD_US - g_currentServoPulseUs) >> 8);
        TL1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(SERVO_PWM_PERIOD_US - g_currentServoPulseUs) & 0xFF);
    } else {
        // Currently LOW, transition to HIGH
        SERVO_CTRL_PIN = 1;
        g_isServoSignalHigh = 1;
        // Set timer for the HIGH duration (pulse width)
        TH1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(g_currentServoPulseUs) >> 8);
        TL1 = (unsigned char)(CALCULATE_TIMER_RELOAD_US(g_currentServoPulseUs) & 0xFF);
    }
    // TF1 is automatically cleared by hardware when ISR is entered for modes 0, 1, 2
}

/* ===================== MAIN APPLICATION LOOP ===================== */
void main(void) {
    system_init();                 // Initialize microcontroller hardware and peripherals
    setupTimer1ForServo();         // Configure and start Timer1 for servo PWM

    EA = 1; // Enable Global Interrupts

    while(1) {
        readParkingBaySensors();           // Check P0 for occupied/free slots
        updateParkingStatusIndicators();   // Update Green/Red LEDs
        displaySlotCountOnSevenSegment(g_availableParkingSlots); // Show count on P2
        // Other non-interrupt driven tasks can be placed here
        // For this system, most actions are interrupt-driven.
    }
}