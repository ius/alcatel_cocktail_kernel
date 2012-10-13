#ifndef MDDI_RY002Z_H
#define MDDI_RY002Z_H 

#define GPIO_RESET 100

struct driver_state_type{
	boolean display_on;
	boolean is_sleep;
	int     bl_level;
};

#endif /* mddi_RY002Z.H */
