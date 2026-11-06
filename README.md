# DRIDLE

DRIDLE.SYS (IDLE.SYS DR-DOS device driver) for Digital Research / Novell BatteryMax by Michal Necasek updated.
- Reference: https://www.os2museum.com/wp/idle-dr-dos/
- Original Mercurial: https://www.os2museum.com/hg/dridle/file/
- R1.11 integrates the enhancements published by gm86 on Win3x.Org (BatteryMAX thread,
  viewtopic.php?t=25683): real timer calibration, floppy-motor nibble check, standard PIT
  command port 043h, optional DRI EMM386.SYS V2.x support, PS/2 mouse IRQ mask, CPU cache
  divider and midnight handling (build the new dridlep.sys with -DPIC_SUP. Forum reference sources are kept under forum-gm86/.
