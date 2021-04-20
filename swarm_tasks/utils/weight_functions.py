import numpy as np



def hyperbolic(t, initial_val=1.5, final_val=0.5, rate=0.001):
	return final_val + (initial_val-final_val)/(1+t*rate)



def linear_trunc(t, initial_val=0.0, final_val=1.0, rate=0.001):
	lin = initial_val + rate*(final_val-initial_val)*t
	min_lin, max_lin = min(initial_val, final_val), max(initial_val, final_val)
	return max(min(lin, max_lin), min_lin)