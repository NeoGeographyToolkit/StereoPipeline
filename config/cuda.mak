# Makerules.

NVCC=nvcc

.cu.cc:
	$(NVCC) $(ASP_CPPFLAGS) --cuda $< -o $@
