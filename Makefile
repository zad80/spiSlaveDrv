# note very well!! the outputed driver name
# MUST HAVE a UNIQUE name , there shall not be 
# a source with the same name, in that case the 
# source will not be included in the build !!!
DRIVERNAME:=spiSlaveDrv
ifeq ($(KERNEL_DIR),)
KERNEL_DIR:= /usr/src/linux
# search the kernel source code
ifeq (,$(wildcard $(KERNEL_DIR)/Kbuild))
KERNEL_DIR:=/usr/src/linux-header-$(shell uname -r)
endif
endif

PWD:=$(shell pwd)
dirs := $(shell find $(PWD) -type d| grep -v "$(KERNEL_DIR)"  | grep -v shell)
srcs := $(shell find $(PWD) -name "*.c" |  grep -v "$(KERNEL_DIR)"|  grep -v shell )
INCL += $(foreach d,$(dirs),$(addprefix -I,$d))
OBJSt = $(foreach c,$(srcs),$(subst .c,.o,$c))
OBJS := $(foreach c,$(OBJSt),$(subst $(PWD)/, ,$c))

# MAKEFLAGS is absolutely neede to build also the other objects specified
# in $(DRIVERNAME)-objs
MAKEFLAGS +=e
export OBJS
export INCL
MODFLAGS += $(INCL)
ifeq ($(KERNELVERSION),)
# we aren't inside the kbuild system so
# go inside it


all:
	$(MAKE) ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} -C $(KERNEL_DIR) SUBDIRS=$(PWD) EXTRA_CFLAGS="$(INCL) -g " modules
clean:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$(PWD) clean
	#rm $(OBJS)

else
$(info INCL= $(INCL) OBJS=$(OBJS) )
obj-m:=$(DRIVERNAME).o 
$(DRIVERNAME)-objs :=$(OBJS)
endif
