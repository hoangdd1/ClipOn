#include<stdio.h>
#include<stdlib.h>

#include "datastruct_stack.h"


void pop(Stack_utill *S)
{
/* If stack size is zero then it is empty. So we cannot pop */
    if(S->size==0)
    {
    //PRINTF("Stack is Empty\n");
    return;
    }
/* Removing an element is equivalent to reducing its size by one */
else
{
 S->elements[S->size-1] = -1;
 S->size--;
}
return;
}

int top(Stack_utill *S)
{
if(S->size==0)
  {
  //PRINTF("Stack is Empty\n");
    return -1; 
  }
  /* Return the topmost element */
  if (S->elements[S->size-1] == -1)
    S->size--;
  return S->elements[S->size-1];
}

int is_stackFull(Stack_utill *S) 
{
  if(S->size == S->capacity)
    return 1;
  else
    return 0;
}

int stack_capacity(Stack_utill *S) 
{
  return S->capacity;
}
int stack_number_of_elements(Stack_utill *S) 
{
  return S->size;
}
void push_lifo(Stack_utill *S,int element)
{
/* If the stack is full, we cannot push an element into it as there is no space for it.*/
if(S->size == S->capacity)
{
 //PRINTF("Stack is Full\n");
;
}
else
{
/* Push an element on the top of it and increase its size by one*/ 
S->elements[S->size++] = element;
}
return;
}

int new_elements = -1;
int post_push_index = -1;
int insert_new_element = 0;
int post_push_slow_fifo(Stack_utill *S)
{
  if ( post_push_index >= 0 )
  {
    S->elements[post_push_index+1] = S->elements[post_push_index];
    post_push_index--;
    insert_new_element = 1;
    return 0; 
  }
  else if (insert_new_element)
  {
      S->elements[0] = new_elements;
      S->size++;     
      insert_new_element = 0;
      return 1;
  }
  return 1;
}


int pre_push_slow_fifo(Stack_utill *S,int element)
{
/* If the stack is full, we cannot push an element into it as there is no space for it.*/
if(S->size == S->capacity)
{
 return 0;
}
else
{
/* Push an element on the tail of it and increase its size by one*/  
  post_push_index = S->size;
  new_elements = element; 
}
return 1;
}

int push_fifo(Stack_utill *S,int element)
{
/* If the stack is full, we cannot push an element into it as there is no space for it.*/
if(S->size == S->capacity)
{
 return 0;
}
else
{
/* Push an element on the tail of it and increase its size by one*/ 
  int index;
  
  for ( index = S->size ; index >= 0 ; index--)
    S->elements[index+1] = S->elements[index];
  
  S->elements[0] = element;
  S->size++;
}
return 1;
}
