ccflags-y+=-Werror
ccflags-y+=-Idrivers/tee/include/linux
ccflags-y+=-Idrivers/tee/include

obj-$(CONFIG_TEE) += tee.o
tee-objs += tee_core.o
tee-objs += tee_shm.o
tee-objs += tee_shm_pool.o
obj-$(CONFIG_OPTEE) += optee/
