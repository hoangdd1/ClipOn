/*
 * sms_message.h
 *
 * Created: 10/9/2015 12:22:26 AM
 *  Author: ofer
 */ 


#ifndef SMS_MESSAGE_H_
#define SMS_MESSAGE_H_

typedef enum
{
	/*  0 - HELP									*/ HP_SMS_MESSAGE,
	/*  1 - STATUS REPORTING						*/ OK_SMS_MESSAGE,
	/*  2 - SET (ACTIVATION) TIME					*/ ST_SMS_MESSAGE,
	/*  3 - SET (ACTIVATION) FREQUENCY				*/ SF_SMS_MESSAGE,
	/*  4 - ERROR EVENT								*/ ER_SMS_MESSAGE,
	/*  5 - CLEANING COMPLETED						*/ CC_SMS_MESSAGE,
	/*  6 - START CLEANING							*/ SC_SMS_MESSAGE,
	/*  7 - SET BATTERY THRESHOLD CLEANING WAGON	*/ CT_SMS_MESSAGE,
	/*  8 - SET BATTERY THRESHOLD TRANSFER WAGON	*/ TT_SMS_MESSAGE,
	/*  9 - ADC DETAILS								*/ AD_SMS_MESSAGE,
	/* 10 - LOCATION DETAILS						*/ LC_SMS_MESSAGE,
	/* 11 - WATER FLOW DETAILS						*/ WF_SMS_MESSAGE,
	/* 12 - BRUSH MOTOR SPEED						*/ PM_SMS_MESSAGE,
	/* 13 - GO FORWARD								*/ GF_SMS_MESSAGE,
	/* 14 - GO BACKWARD								*/ GB_SMS_MESSAGE,
	/* 15 - STOP CLEANING PROCESS					*/ SP_SMS_MESSAGE,
	/* 16 - RESET SYSTEM							*/ RS_SMS_MESSAGE,
	/* 17 - PAUSE SYSTEM							*/ PA_SMS_MESSAGE,
	/* 18 - CONTINUE SYSTEM							*/ CO_SMS_MESSAGE,
	/* 19 - DEBUGING								*/ DB_SMS_MESSAGE,
	/* 20 - ADD (PHONE) NUMBER						*/ AN_SMS_MESSAGE,
	/* 21 - REMOVE (PHONE) NUMBER					*/ RN_SMS_MESSAGE,
	/* 22 - LIST (OF PHONE) NUMBERS					*/ LN_SMS_MESSAGE,
	
	/* 23 */ NUMBER_OF_SMS_MESSAGE
} SMS_MESSAGE;

char * send_help_sms_message();
char *get_help_string();
void sms_message_loop();
void send_sms_to_administrator( char* sms_message );
void send_sms_message_to_sender( SMS_MESSAGE sms_message );
void sms_message_reception_interrupt_enable();
void sms_message_reset_number_of_waiting_messages();

#endif /* SMS_MESSAGE_H_ */
