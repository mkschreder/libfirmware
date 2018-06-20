#include "atomic.h"

__atomic_t atomic_set(atomic_t *self, atomic_t other){
	// this will loop until the set has been successful without any write
	// between reading the old value and setting the new one. Thus guaranteeing
	// proper ordering.
	atomic_t prev = *self;
	while(*self != other) {
		prev = __sync_val_compare_and_swap(self, *self, other);
	}
	return prev;
}

