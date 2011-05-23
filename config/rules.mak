# __BEGIN_LICENSE__
# Copyright (C) 2006-2011 United States Government as represented by
# the Administrator of the National Aeronautics and Space Administration.
# All Rights Reserved.
# __END_LICENSE__


newtest:
	@if test -z "$(NAME)"; then echo "run make NAME=TestName [MODULE=ModuleName] newtest"; else $(top_srcdir)/scripts/create-test.sh $(NAME) $(MODULE); fi

.PHONY: newtest

SUFFIXES = .totallyfakeplaceholder
include $(top_srcdir)/thirdparty/autotroll.mak

# vim: filetype=automake:
