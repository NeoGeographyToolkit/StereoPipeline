# __BEGIN_LICENSE__
# Copyright (C) 2006-2011 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


########################################################################
# rules for compiling Google Protocol Buffers (using protoc)
########################################################################

SUFFIXES += .proto .pb.cc

PROTOC = protoc
PROTOC_ARGS = -I. --cpp_out=.

.proto.pb.cc :
	$(AM_V_GEN)$(PROTOC) $(PROTOC_ARGS) $<
