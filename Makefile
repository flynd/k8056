all: k8056.hex

k8056.hex: k8056.asm
	gpasm $<

flash: k8056.hex
	pk2cmd -P PIC16F616 -F $< -M

sim: k8056.hex
	gpsim -s k8056.cod -c k8056.stc

clean:
	$(RM) k8056.cod k8056.hex k8056.lst
