#ifndef _DATASTRUCT_STACK_H_
#define _DATASTRUCT_STACK_H_

#include <stdint.h>
#include <stdio.h>

/* Stack has three properties. capacity stands for the maximum number of elements stack can hold.
   Size stands for the current size of the stack and elements is the array of elements */
typedef struct _STACK_utill
{
  int capacity;
  int size;
  int *elements;
}Stack_utill;

  void pop(Stack_utill *S);
  int top(Stack_utill *S);
  int is_stackFull(Stack_utill *S);  
  int stack_capacity(Stack_utill *S); 
  int stack_number_of_elements(Stack_utill *S);
  void push_lifo(Stack_utill *S,int element);
  int push_fifo(Stack_utill *S,int element);
  int post_push_slow_fifo(Stack_utill *S);
  int pre_push_slow_fifo(Stack_utill *S,int element);  

#endif /* _DATASTRUCT_STACK_H_ */