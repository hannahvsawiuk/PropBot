#include "array_function.h"
#include <stdlib.h>
#include <stdio.h>

void initArray(Array *a, size_t initialSize) {
	a->array = (int *)malloc(initialSize * sizeof(int));
	a->used = 0;
	a->size = initialSize;
}

void insertArray(Array *a, int element) {
	if (a->used == a->size) {
		a->size += 1;
		a->array = (int *)realloc(a->array, a->size * sizeof(int));
	}
	a->array[a->used++] = element;
}

void freeArray(Array *a) {
	free(a->array);
	a->array = NULL;
	a->used = a->size = 0;
}

// function to sort the array in ascending order
void Array_sort(int *array, size_t n)
{
	int i = 0, j = 0, temp = 0;

	for (i = 0; i<n; i++){
		for (j = 0; j<n - 1; j++){
			if (array[j]>array[j + 1]){
				temp = array[j];
				array[j] = array[j + 1];
				array[j + 1] = temp;
			}
		}
	}
}

// function to calculate the median of the array
int Find_median(int array[], size_t n)
{
	int median = 0;

	// if number of elements are even
	if (n % 2 == 0)
		median = (array[(n - 1) / 2] + array[n / 2]) / 2;
	// if number of elements are odd
	else
		median = array[n / 2];

	return median;
}