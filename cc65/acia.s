; ---------------------------------------------------------------------------
; acia.s
; icestick_6502 ACIA interface routines
; 03-04-19 E. Brombaugh
; ---------------------------------------------------------------------------
;
; Write a string to the ACIA TX

.export         _acia_tx_str
.export         _acia_tx_chr
.export         _acia_rx_chr
.export         _test_instructions
.exportzp       _acia_data: near

.include  "fpga.inc"

.zeropage

_acia_data:     .res 2, $00        ;  Reserve a local zero page pointer

.segment  "CODE"

; ---------------------------------------------------------------------------
; send a string to the ACIA

.proc _acia_tx_str: near

; ---------------------------------------------------------------------------
; Store pointer to zero page memory and load first character

        sta     _acia_data       ;  Set zero page pointer to string address
        stx     _acia_data+1     ;    (pointer passed in via the A/X registers)
        ldy     #00              ;  Initialize Y to 0
        lda     (_acia_data),y   ;  Load first character

; ---------------------------------------------------------------------------
; Main loop:  read data and store to FIFO until \0 is encountered

loop:   jsr     _acia_tx_chr     ;  Loop:  send char to ACIA
        iny                      ;         Increment Y index
        lda     (_acia_data),y   ;         Get next character
        bne     loop             ;         If character == 0, exit loop
        rts                      ;  Return
.endproc

; ---------------------------------------------------------------------------
; wait for TX empty and send single character to ACIA

.proc _acia_tx_chr: near

        pha                      ; temp save char to send
txw:    lda      ACIA_CTRL       ; wait for TX empty
        and      #$02
        beq      txw
        pla                      ; restore char
        sta      ACIA_DATA       ; send
        rts

.endproc

; ---------------------------------------------------------------------------
; wait for RX full and get single character from ACIA

.proc _acia_rx_chr: near

rxw:    lda      ACIA_CTRL       ; wait for RX full
        and      #$01
        beq      rxw
        lda      ACIA_DATA       ; receive
        rts

.endproc

.proc _test_instructions: near

        lda      $12
        pha
        lda      #$00
        sta      $12
        smb2     $12
        smb7     $12
        lda      $12
        cmp      #$84
        bne      error
        lda      #$FF
        sta      $12
        rmb0     $12
        rmb6     $12
        lda      $12
        cmp      #$BE
        bne      error
        bbr7     $12,error
        bbs6     $12,error
        bbr5     $12,error
        bbr4     $12,error
        bbr3     $12,error
        bbr2     $12,error
        bbr1     $12,error
        bbs0     $12,error
        bbs7     $12,s1
        bra      error
s1:     bbr6     $12,s2
        bra      error
s2:     bbs5     $12,s3
        bra      error
s3:     bbs4     $12,s4
        bra      error
s4:     bbs3     $12,s5
        bra      error
s5:     bbs2     $12,s6
        bra      error
s6:     bbs1     $12,s7
        bra      error
s7:     bbr0     $12,s8
        bra      error
s8:     pla
        sta      $12
        lda      #$01
        rts
error:  pla
        sta      $12
        lda      #$00
        rts

.endproc
