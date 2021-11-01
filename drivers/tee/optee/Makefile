ccflags-y+=-Werror
ccflags-y+=-Idrivers/tee/include/linux
ccflags-y+=-Idrivers/tee/include

obj-$(CONFIG_OPTEE) += optee.o
optee-objs += core.o
optee-objs += call.o
optee-objs += rpc.o
optee-objs += supp.o
optee-objs += smccc-call.o
