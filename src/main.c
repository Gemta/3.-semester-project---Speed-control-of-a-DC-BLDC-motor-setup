#define F_CPU 16000000UL

#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <lcd.h>
#include <i2cmaster.h>

// Pins used for circuit:
// - PD6 for Timer0 output (duty cycle)
// - PD3 for Timer1 input (Hall-sensor)
// - PC0 for adc input (potentiometer output)
// - 5V output


// Variables
volatile int Rotations_total = 0;
volatile int Rotations_old = 0;
volatile float Rotations_diff = 0;

volatile float velocity = 0;
volatile float velocity_prev = 0;

volatile float RPM_counter = 0;
float RPM_target = 0;
float RPM_max = 3650;

volatile int sensor_count;

volatile int microsec_counter = 0;
volatile int microsec_old = 0;
volatile float microsec_diff = 0;

volatile int millisec_counter = 0;
volatile int millisec_old = 0;
volatile float millisec_diff = 0;

volatile float e = 0;
volatile float e_last = 0;
volatile float e_cum, e_rate;
volatile int u = 0;
uint16_t mc = 0;

volatile int update_flag = 0;


// Function prototypes
void motor_control_init(); // Function to initialize the control of the motor with timer0.
void interrupt_init(); // Function to initialize interrupts on timer1. 

void motor_control_running(uint16_t duty_cycle); // Function to control the speed of the motor while it is running.
uint16_t pot_read(uint8_t adc_channel); // Function to read the output of the ADC.
void update_display(float dt, float target_RPM, float count_RPM); // Function to update the LCD display.
int pid(float target, float meas, float kp, float ki, float kd); // Function for the PID-controller.


// Main Function
int main(void){

    // Pin configuration
    DDRD |= (1 << PD6); //Sets PD6 and PD3 as outputs (0 = input, 1 = ouput).

    // ADC setup
    uint16_t adc_result; // ADC output.
    ADMUX = (1 << REFS0); //Selecting Vref = AVcc.
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN); //Setting the prescaler.

    // Duty cycle starting value
    OCR0A = 0x00; // 0% Duty cycle, motor turned off.

    // Variables
    float dt_display; // Duty-cycle to display on the LCD.
    float dt_calc; // Duty-cycle to be used for calculations.

    float kp = 1.25;      // Propertional gain.
    float ki = 0.0008;   // Integral gain.
    float kd = 5;       // Derivative gain.

    // Initiallisation
    motor_control_init();
    interrupt_init();
    i2c_init();
    LCD_init();

    // Main program
    while(1){

        adc_result = pot_read(0)/4; // Result from potentiometer = reference value for speed control. 
        
        if(adc_result < 26){ // If below 10% duty-cycle, hard set the ADC, PID, RPM counter and RPM target to 0 to turn off the motor.
            adc_result = 0;
            mc = 0;
            RPM_counter = 0; // Sets the RPM-counter to 0, so old RPM measurement is not displayed.
            RPM_target = 0; // Sets the RPM to target to 0, so old RPM target is not displayed.
            motor_control_running(0); // Changes the duty-cycle to 0, so it turns off.
            LCD_set_cursor(0,3);
            printf("Motor turned off");
        }else{
            // Calculates the duty-cycle based on the ADC output.
            if(adc_result > 229){ // If above 90% duty-cycle, hard set it to 90%.
                dt_calc = 229/255.0;
            }else if(adc_result < 26){ // If below  10% duty-cycle, hardset it to 0%.
                dt_calc = 0;
            }else{ // Calculates the duty-cycle when between 10% and 90%.
                dt_calc = adc_result/255.0;
            }

            RPM_target = RPM_max*dt_calc; // Calculates the RPM to target/what the RPM should be.
            mc = pid(RPM_target, RPM_counter, kp, ki, kd); // Gets the error correction from the PID-controller.
            
            if(mc < 0){mc = 0;} // Prevents the mc from being negative.

            // Sets the motor speed based on the ADC output.
            if(mc > 229){ // If above 90% duty-cycle, hard set it to 229.
                mc = 229; // Hard sets the PID signal to 90% duty-cycle.
                motor_control_running(mc); // Changes the duty-cycle to the input argument.
                LCD_set_cursor(0,3);
                printf("Duty cycle at 90%%");
            }else{ // Sets the motor speed and calculates the target RPM, when between 10% and 90% duty-cycle.
                motor_control_running(mc); // Changes the duty-cycle to the input argument.
                LCD_set_cursor(0,3);
                printf("                      ");
            }

        }
        
        // Updates the LCD display and calculates the duty-cycle in percent, when the update flag is set to 1.
        if(update_flag == 1){
            dt_display = mc/2.55; // Calculating the duty-cycle in percent.
            update_display(dt_display, RPM_target, RPM_counter); // Update function for the display.
        }
        
    }

    return 0;
}

// Functions
void motor_control_init(){
    // Timer/counter 0
    // Operation mode       = Fast PWM
    // Prescaler            = 1 (frequency = 62.5 KHz)

    TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00); //Sets the operation mode to fast PWM in non-inverting mode
    TCCR0B |= (1 << CS00); //Sets the prescaler to 1 and starts the timer.

}

void interrupt_init(){
    // Timer/counter 1
    // Operation mode       = CTC
    // TOP count            = 1599
    // Prescaler            = 1
    // Timer count          = 100 microsecond
    // Interrupt pin, Int1  = PD3 

    //Timer1 configuration
    TCCR1B |= (1 << WGM12); //Sets Timer mode to CTC.
    OCR1A = 1599; //Sets the value to count to.
    TIMSK1 |= (1 << OCIE1A); //Set the ISR_COMPA_vect.
    TCCR1B |= (1 << CS10); //Set the prescaler to 1 and start the timer.

    //Int1 configuration
    DDRD &= ~(1 << DDD3); // Clear the PD3 pin, sets it to input
    PORTD |= (1 << PORTD3); //Turn on the pull-up for PD3.
    EICRA |= (1 << ISC11); // set INT1 to trigger on falling edge.
    EIMSK |= (1 << INT1); // Turns on interrupt for INT1.

    sei(); //Enables interrupts.
    
}

uint16_t pot_read(uint8_t adc_channel){

    ADMUX &= 0xf0; //Clear any previously used channels, but keep the internal reference

    ADMUX |= adc_channel; //Set the channel
    ADCSRA |= (1 << ADSC); //Start conversion

    while( (ADCSRA & (1 << ADSC)) ); //Wait until conversion is complete (while ADSC is 1)

    return ADC; //Return the conversion as an unsigned 16-bit integer

}

void motor_control_running(uint16_t duty_cycle){

    OCR0A = duty_cycle; // Sets the duty-cycle

}

int pid(float target, float meas, float kp, float ki, float kd){

    millisec_diff = millisec_counter - millisec_old; // Difference in time in millisecond.

    e = target - meas; // Propertional part: Calculates the error between the target RPM and the measured RPM.
    e_cum += e * millisec_diff; // Integral part: Calculates the cumulative error.
    e_rate = (e-e_last)/millisec_diff; // Derivative part: Calculates the change of the error.

    float kprop = kp*e; // Function for the propertinal part.
    float kint = ki*e_cum; // Function for the integral part.
    float kdev = kd*e_rate; // Function for the derivative part.

    u = (kprop + kint + kdev)/14.3137; // The output of the PID, scaled to be a maximum of 255.
    
    millisec_old = millisec_counter; // Sets the old measurement equal to the current.
    e_last = e; // Sets the old measurement equal to the current.

    return u;
    
}

void update_display(float dt, float target_RPM, float count_RPM){
    
    LCD_set_cursor(0,0); // Sets the position of the cursor to the second line.
    printf("Target RPM: %1.1f        ", target_RPM); // Prints the target RPM to the LCD.

    LCD_set_cursor(0,1); // Sets the position of the cursor to the first line.
    printf("Duty cycle: %1.1f %%         ", dt); // Prints the duty-cycle to the LCD.

    LCD_set_cursor(0,2); // Sets the position of the cursor to the third line.
    printf("RPM: %1.1f            ", count_RPM); // Prints the RPM to the LCD.

    update_flag = 0;
}


// Interrupts
ISR(TIMER1_COMPA_vect){

    microsec_counter++; // Add 1 to the total number of 100 microseconds, that has elapsed.

    // Activates 1000 times a second, or once every 1ms
    if(microsec_counter % 10 == 0){
        millisec_counter++;
    }

    // Activates 10 times a second, or once every 100ms
    if(millisec_counter % 100 == 0){
        update_flag = 1;
    }

    // Activates 2000 times a second, or once every 5ms
    if(microsec_counter % 50 == 0){
        
        // Difference in rotations from last measurement
        Rotations_diff = Rotations_total - Rotations_old;

        // Calculates the velocity in rotations per second
        velocity = (Rotations_diff/microsec_diff)*10000;

        if(velocity > 0){
            // Calculates the RPM.
            RPM_counter = velocity*60; 
            
            // Sets the old measurement equal to the new.
            Rotations_old = Rotations_total; 
        }
    
    }

}

ISR(INT1_vect){

    sensor_count++; // Add 1 to the total number of external interrupts.

    if(sensor_count % 2 == 0){
        // Add 1 to the total number of rotations.
        Rotations_total++; 

        // Difference in milliseconds from last interrupt.
        microsec_diff = (microsec_counter - microsec_old); 

        // Sets the old measurement equal to the new.
        microsec_old = microsec_counter; 

    }    

}


