#ifndef array_function_H
#define array_function_H
#include <Arduino.h>

typedef struct {
	int *array;
	size_t used;
	size_t size;
} Array;

void initArray(Array *a, size_t initialSize);

void insertArray(Array *a, int element);

void freeArray(Array *a);

void Array_sort(int *array, size_t n);

int Find_median(int array[], size_t n);


#endif // array_function_H
