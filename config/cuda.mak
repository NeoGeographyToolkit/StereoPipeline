# Makerules.

SUFFIXES += .cu.o .cu.lo .cu

.cu.o:
	nvcc $< -c $@

.cu.lo:
	nvcc $< -c $@

.cu:
	nvcc $< -o $@

