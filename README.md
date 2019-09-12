# md_boot
Drop Bootloader based on QMK project

  Not every file is a direct copy from QMK project, although through use of define MD_BOOTLOADER efforts were made to keep them similar
  Changes were usually cherry picked from main QMK as project progressed
  Directory "/md_boot/keyboards/massdrop" should be a direct copy of the "/qmk_firmware/keyboards/massdrop" folder

  Make:
    make kb=alt
    make kb=ctrl
    make kb=shift
    make kb=rocketeer
  Output to ".build" directory
    massdrop_alt_bootloader.bin             - For factory programming process
    massdrop_ctrl_bootloader.bin            - For factory programming process
    massdrop_shift_bootloader.bin           - For factory programming process
    massdrop_rocketeer_bootloader.bin       - For factory programming process
  Bootloader to be written at byte 0x0 of flash with maximum size 0x4000 bytes


