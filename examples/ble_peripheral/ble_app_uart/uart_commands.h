typedef enum 
{
    NOT_EXIST = 0,
    EXIST     = 1
} EXISTANCE;

EXISTANCE process_uart_command(char * input_msg, int input_length , char * output_msg , int *output_length);
extern void push_ping_packets();


