all : dridle.sys tstidle.exe

dridle.sys : dridle.exe
        exe2bin -q $< $@

dridle.exe : dridle.obj
        wlink sys dos op quiet, map disable 1014, 1023 file $<

dridle.obj : dridle.asm driver.equ drmacros.equ idle.equ reqhdr.equ
!ifdef USE_MASM
        masm32 /DVBOX_CPU_HALT dridle.asm;
!else
        wasm -q -wx -DVBOX_CPU_HALT dridle.asm
!endif


tstidle.exe : tstidle.c
        wcl -q -wx -l=dos $<

clean: .symbolic
        rm -f *.obj
        rm -f *.exe
        rm -f *.sys
        rm -f *.map
        rm -f *.err
