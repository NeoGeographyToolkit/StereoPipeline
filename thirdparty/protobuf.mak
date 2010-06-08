########################################################################
# rules for compiling Google Protocol Buffers (using protoc)
########################################################################

SUFFIXES += .proto .pb.cc

PROTOC = protoc
PROTOC_ARGS = -I. --cpp_out=.

.proto.pb.cc :
	$(AM_V_GEN)$(PROTOC) $(PROTOC_ARGS) $<
