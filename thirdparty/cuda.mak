# __BEGIN_LICENSE__
# Copyright (C) 2006-2011 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


# Makerules.

NVCC=nvcc

.cu.cc:
	$(NVCC) --cuda $< -o $@
#	$(NVCC) $(ASP_CPPFLAGS) --cuda $< -o $@
