Fuentes de referencia publicados por gm86 en el foro Win3x.Org
==============================================================

Hilo : BatteryMAX
URL  : http://www.win3x.org/win3board/viewtopic.php?t=25683
Autor: gm86 (ultimo volcado del post: actualizacion del 9/10/2026, 2:00)

Estos ficheros son el material ORIGINAL tal y como fue publicado en el hilo;
NO forman parte del build de DRIDLE. La version integrada de este repositorio
es dridle.asm + idle.equ (ver ../CAMBIOS-WIN3X.ORG.md para el detalle).

IDLE86.ASM    Fuente principal mejorado por gm86 sobre el IDLE86.ASM original
              de Digital Research (patente BatteryMAX US 5,355,501). Incluye el
              relleno del blanco con CALL TIMER_READ, TEST ...,0Fh del disquete,
              el soporte ifdef pic (callback V86 de EMM386.SYS 2.x de DR DOS 6,
              divisor por familia de CPU, medianoche, raton PS/2) y los puertos
              emulados 0EDC0h/0EDC1h.
IDLE.EQU      Su IDLE.EQU correspondiente (markers del area de datos int 60h,
              puertos emulados, offsets).
DRMACROS.EQU / REQHDR.EQU / DRIVER.EQU   Copias del original de DR (sin
              modificar; solo difieren en espacios del que usa este repo).
BUILD.BAT     Sus instrucciones de ensamblado con MASM (unidad I: = LASTDRIVE
              minimo):
                CD I:=.
                MASM IDLE86;
                CD I:=
                LINK IDLE86;
                EXE2BIN IDLE86 IDLE86.SYS
                TOUCH /T:2:0 IDLE86.SYS
              (CD equivale a SUBST bajo DR DOS.)

patches-dr-dos/ : parches binarios publicados en el mismo hilo para otros
programas de DR DOS; NO pertenecen al driver IDLE y no se integran:
  403F-MOTOR.TXT        Documentacion del byte 40:3Fh (estado motores disquete)
                        que justifica el TEST ...,0Fh del driver.
  SUPERPCK-FR.TXT       Correccion del texto en frances de Super PC-Kwik
                        (PKLITE 1.12 + SID).
  SUPERPCK-IT.TXT       Lo mismo para la version italiana ("u" y " " en vez de
                        ^U y 0).
  KEYB-206.TXT          Parche al cuelgue de KEYB 2.06 con funciones desconocidas
                        (inspirado en la 2.08 de 3/93, WordPerfect 6).
  IBMDOS-NOP.TXT        NOP en IBMDOS.COM.
  DISPLAY.SYS-STACK.TXT Gestion de pila de DISPLAY.SYS (dos versiones).
