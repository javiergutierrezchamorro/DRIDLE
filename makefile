all : dridle.sys dridlev.sys dridle3.sys dridle3v.sys tstidle.exe

dridle.sys : dridle.exe
        exe2bin -q $< $@

dridlev.sys : dridlev.exe
        exe2bin -q $< $@

dridle3.sys : dridle3.exe
        exe2bin -q $< $@

dridle3v.sys : dridle3v.exe
        exe2bin -q $< $@


dridle.exe : dridle.obj
        wlink sys dos op quiet, map disable 1014, 1023 file $<

dridlev.exe : dridlev.obj
        wlink sys dos op quiet, map disable 1014, 1023 file $<

dridle3.exe : dridle3.obj
        wlink sys dos op quiet, map disable 1014, 1023 file $<

dridle3v.exe : dridle3v.obj
        wlink sys dos op quiet, map disable 1014, 1023 file $<


dridle.obj : dridle.asm driver.equ drmacros.equ idle.equ reqhdr.equ
!ifdef USE_MASM
	masm32 dridle.asm;
!else
	wasm -q -wx dridle.asm
!endif

dridlev.obj : dridle.asm driver.equ drmacros.equ idle.equ reqhdr.equ
!ifdef USE_MASM
        masm32 /DVBOX_CPU_HALT dridle.asm /Fo dridlev.obj;
!else
        wasm -q -wx -DVBOX_CPU_HALT dridle.asm -fo=dridlev.obj
!endif

dridle3.obj : dridle.asm driver.equ drmacros.equ idle.equ reqhdr.equ
!ifdef USE_MASM
	masm32 /3 dridle.asm /Fo dridle3.obj;
!else
	wasm -3 -q -wx dridle.asm /fo=dridle3.obj
!endif

dridle3v.obj : dridle.asm driver.equ drmacros.equ idle.equ reqhdr.equ
!ifdef USE_MASM
        masm32 /DVBOX_CPU_HALT dridle.asm /3 /Fo dridle3v.obj;
!else
        wasm -3 -q -wx -DVBOX_CPU_HALT dridle.asm -fo=dridle3v.obj
!endif




tstidle.exe : tstidle.c
        wcl -q -wx -l=dos $<

clean: .symbolic
        rm -f *.obj
        rm -f *.exe
        rm -f *.sys
        rm -f *.map
        rm -f *.err
