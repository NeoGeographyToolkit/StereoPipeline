# Makerules.

NVCC=nvcc

.cu.cc:
	$(NVCC) --cuda $< -o $@
#	$(NVCC) $(ASP_CPPFLAGS) --cuda $< -o $@
