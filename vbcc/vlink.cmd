STACKLEN = 0x400;

MEMORY
{
  zero    : org = 0x0000, len = 256
  ram     : org = 0x0200, len = 0xD000-STACKLEN-0x200
  rom     : org = 0xE000, len = 0x2000-6
  vectors : org = 0xFFFA, len = 6
}

SECTIONS
{
  text           : {*(text)}    >rom
  vectors        : {*(vectors)} >vectors
  .dtors         : {*(.dtors)}  >rom
  .ctors         : {*(.ctors)}  >rom
  rodata         : {*(rodata)}  >rom
  data           : {*(data)}    >ram AT>rom
  bss (NOLOAD)   : {*(bss)}     >ram
  zpage (NOLOAD) : {*(zpage) *(zp1) *(zp2)} >zero

  __STACK = 0xD000-STACKLEN;
  __DS    = ADDR(data);
  __DE    = ADDR(data) + SIZEOF(data);
  __DC    = LOADADDR(data);
  __BS    = ADDR(bss);
  __BE    = ADDR(bss) + SIZEOF(bss);

  ___heap    = ADDR(bss) + SIZEOF(bss);
  ___heapend = 0xCFFF-STACKLEN;
}
