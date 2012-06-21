/*
 * Heavily based on VirtualSerial demo. Original copyright (for that demo) follows.
 *
             LUFA Library
     Copyright (C) Dean Camera, 2012.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2012  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaim all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/
#include <avr/eeprom.h>

#include "thermostat.h"

#include "onewire.h"
#include "ds18x20.h"

#define MAXSENSORS 2

uint8_t cutoff_temp = 0, new_cutoff_temp = 0;
int16_t most_recent_temperature = 0;
uint8_t temperature_result = DS18X20_ERROR;

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE];

uint16_t uptime_ms = 0;

// Changed this to a periodic timer every 1/10th of a second (and prescaler of /64)
#define TIMER1_DELAY (65536 - 25000)
#define MS_PER_TICK 100

#define DEFAULT_CUTOFF_TEMPERATURE_C 37

#define OUTPUT_PIN_PORT PORTD
#define OUTPUT_PIN_DDR DDRD
#define OUTPUT_PIN_MASK 0b00000001
#define OUTPUT_PIN_INVMASK (~OUTPUT_PIN_MASK)

// Don't switch (from on to off or vice versa) more frequently than this.
#define CHANGE_DELAY_MS 2000

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = 0,

				.DataINEndpointNumber           = CDC_TX_EPNUM,
				.DataINEndpointSize             = CDC_TXRX_EPSIZE,
				.DataINEndpointDoubleBank       = false,

				.DataOUTEndpointNumber          = CDC_RX_EPNUM,
				.DataOUTEndpointSize            = CDC_TXRX_EPSIZE,
				.DataOUTEndpointDoubleBank      = false,

				.NotificationEndpointNumber     = CDC_NOTIFICATION_EPNUM,
				.NotificationEndpointSize       = CDC_NOTIFICATION_EPSIZE,
				.NotificationEndpointDoubleBank = false,
			},
	};

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs
 */
FILE USBSerialStream;

static uint8_t search_sensors(void)
{
	uint8_t i;
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t diff, nSensors;
	
	ow_reset();

	nSensors = 0;
	
	diff = OW_SEARCH_FIRST;
	while ( diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ) {
		DS18X20_find_sensor( &diff, &id[0] );
		
		if( diff == OW_PRESENCE_ERR ) {
			break;
		}
		
		if( diff == OW_DATA_ERR ) {
			break;
		}
		
		for ( i=0; i < OW_ROMCODE_SIZE; i++ )
			gSensorIDs[nSensors][i] = id[i];
		
		nSensors++;
	}
	//fprintf(&USBSerialStream, "Sensors found: %d\r\n", nSensors);
	
	return nSensors;
}

static void show_temp(void)
{
	if(temperature_result != DS18X20_OK) {
		fprintf(&USBSerialStream, "E (%d)\r\n", cutoff_temp);
	}else{
		fprintf(&USBSerialStream, "%d (%d)\r\n", most_recent_temperature / 10, cutoff_temp);
	}
}

static void show_counter(void)
{
	fprintf(&USBSerialStream, "Counter: %d\r\n", uptime_ms);
}

static void
showOptions(void)
{
	fputs("t    temperature\r\n", &USBSerialStream);
	fputs("c    isr counter\r\n", &USBSerialStream);
	fputs("0-9  cutoff\r\n", &USBSerialStream);
	fputs("s    set cutoff\r\n", &USBSerialStream);
	fputs("o    on\r\n", &USBSerialStream);
	fputs("O    off\r\n", &USBSerialStream);
}

static void
set_newtemp_frommenu(int byte)
{
	// lsr 1 in base 10
	//
	uint8_t tens = new_cutoff_temp % 10;
	new_cutoff_temp = (tens * 10) + (byte - '0');
}

static void
set_temp_from_newtemp(void)
{
	cutoff_temp = new_cutoff_temp;

	fprintf(&USBSerialStream, "Cutoff: %d\r\n", cutoff_temp);

	eeprom_write_byte((void *)0, cutoff_temp);
}

static void
set_output(uint8_t val)
{
	if(val) {
		OUTPUT_PIN_PORT |= OUTPUT_PIN_MASK;
		LEDs_TurnOnLEDs(LEDS_LED2);
	} else {
		OUTPUT_PIN_PORT &= OUTPUT_PIN_INVMASK;
		LEDs_TurnOffLEDs(LEDS_LED2);
	}
}

static void
doMenu(int byte)
{
	if(byte == '\n' || byte == '?') {
		showOptions();
	} else if (byte == 't') {
		show_temp();
	} else if (byte == 'c') {
		show_counter();
	} else if (byte >= '0' && byte <= '9') {
		set_newtemp_frommenu(byte);
	} else if (byte == 's') {
		set_temp_from_newtemp();
	} else if (byte == 'o') {
		set_output(1);
	} else if (byte == 'O') {
		set_output(0);
	/*} else if (byte == 'd') {
		disk_test();
	*/
	} else {
		fputs("\r\n?\r\n", &USBSerialStream);
	}
}


/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	uint8_t output_is_on = 0;
	uint16_t last_change = 0;

	SetupHardware();

	cutoff_temp = eeprom_read_byte((void *)0);

	// New chips
	if(cutoff_temp == 255) {
		cutoff_temp = DEFAULT_CUTOFF_TEMPERATURE_C;
		eeprom_write_byte((void *)0, cutoff_temp);
	}

	/* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	sei();

	int16_t byte;
	int16_t last_temp_measurement_time;
	int therm_idx;

	search_sensors();

	therm_idx = gSensorIDs[0][0];
	last_temp_measurement_time = uptime_ms + (DS18B20_TCONV_12BIT * 2);

	for (;;)
	{
		byte = CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);
		if(byte > 0) {
			doMenu(byte);
		}

		if (uptime_ms - last_temp_measurement_time > DS18B20_TCONV_12BIT) {
			// Store the most recent temperature and start a new reading.
			temperature_result = DS18X20_read_decicelsius_single(therm_idx, &most_recent_temperature );
			DS18X20_start_meas(DS18X20_POWER_EXTERN, NULL );
			last_temp_measurement_time = uptime_ms;
		}

		// Do thermostat.
		if( (uptime_ms - last_change > CHANGE_DELAY_MS) && (temperature_result == DS18X20_OK)) {
			uint8_t new_output_is_on = ( (most_recent_temperature / 10) < cutoff_temp);

			if(new_output_is_on != output_is_on) {
				output_is_on = new_output_is_on;

				set_output(output_is_on);

				last_change = uptime_ms;
			}
		}

		/* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
		//CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface);

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		USB_USBTask();
	}
}

ISR(TIMER1_OVF_vect)
{
	TCNT1 = TIMER1_DELAY;
	uptime_ms += MS_PER_TICK;

	//disk_timerproc();
}

void
initInterrupt(void)
{
	/* Prescaler = 8 */
	//TCCR1B |= 2;

	/* Prescaler = 64 */
	TCCR1B |= 3;

	TCNT1 = TIMER1_DELAY;
	
	/* Overflow */
    TIMSK1 |= (1 << TOIE1);
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);

#if EXTERNAL_EEPROM == 1
	i2c_init();
#endif

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
	Buttons_Init();
	initInterrupt();

	// Make temperature control port an output
	OUTPUT_PIN_DDR |= OUTPUT_PIN_MASK;

	// And set it off for now.
	OUTPUT_PIN_PORT &= OUTPUT_PIN_INVMASK;
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

