STM8S_Lib=/usr/local/stm8lib
CC=sdcc
#CFLAGS=-c -mstm8 -DSTM8S103 -I./inc -I$(STM8S_Lib)/inc --verbose #--debug
#LDFLAGS=-mstm8 stm8.lib stm8s.lib -L$(STM8S_Lib)/src
CFLAGS=-mstm8 -DSTM8S103 -I./inc -I$(STM8S_Lib)/inc --verbose --std-c11
LDFLAGS=-mstm8 -L$(STM8S_Lib)/src --std-c11
SOURCES=main.c
OBJECTS=$(SOURCES:.c=.o)
OBJECTS_LINK=$(SOURCES:.c=.rel)
FLASHFILE=main.ihx
FLASHFILE_HEX=main.hex

.PHONY: all clean flash

all: $(SOURCES) $(FLASHFILE)

$(FLASHFILE): $(OBJECTS)
	#mkdir -p tmp
	$(CC) $(LDFLAGS) $(OBJECTS_LINK) -o $@
	#packihx $(FLASHFILE) > $(FLASHFILE_HEX)
	#cd tmp && sdcc -I /usr/local/stm8s-sdcc/inc -lstm8 -mstm8 --verbose --out-fmt-ihx $(CFLAGS) $(LDFLAGS) ../main.c /usr/local/stm8s-sdcc/lib/lib_stm8s103_sdcc.lib

clean:
	rm -f *.rel
	rm -f *.asm
	rm -f *.lst
	rm -f *.rst
	rm -f *.sym
	rm -f *.map
	rm -f *.lk
	rm -f *.hex
	rm -f *.ihx
	rm -f *.cdb

flash: 
	stm8flash -cstlinkv2 -pstm8s103f3 -w $(FLASHFILE)