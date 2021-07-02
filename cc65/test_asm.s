; ---------------------------------------------------------------------------
; test_asm.s
; ---------------------------------------------------------------------------

.export         _test_asm

.segment  "CODE"

.proc _test_asm: near
        ldx     #0
        inx
        inx
        jsr     test_indjmp
        jsr     test_xindjmp
        jsr     test_indjsr
        jsr     test_xindjsr
        jsr     test_word_ops
.endproc

.proc test_indjmp: near
        jmp     (indjump)
.endproc

.proc test_xindjmp: near
        jmp     (xindjump, X)
.endproc

.proc test_indjsr: near
        jsr     (indjump)
.endproc

.proc test_xindjsr: near
        jsr     (xindjump, X)
.endproc

.proc test_word_ops: near
        lda     #$FE
        sta     value16
        lda     #$FF
        sta     value16+1
        inw     value16
        inw     value16
        inw     value16
        lda     #$55
        sta     value16
        lda     #$01
        sta     value16+1
        asw     value16
        lda     #$FE
        sta     value16
        lda     #$55
        sta     value16+1
        inw     value16
        inw     value16
        inw     value16
        dew     value16
        dew     value16
        dew     value16
        lda     #>value
        pha
        lda     #<value
        pha
        phw     #value
        jsr     test_prog
        lda     #>value
        pha
        lda     #<value
        pha
        phw     ptr
        bsr     test_prog
        rts
.endproc

.proc test_prog: near
        ldy     #0
        lda     (5, sp), y
        ina
        sta     (5, sp), y
        iny
        lda     (3, sp), y
        ina
        sta     (3, sp), y
        rtn     #4
.endproc

.proc jmpaddr: near
        rts
.endproc

.data

value:  .byte $12
        .byte $34

ptr:    .word value

indjump: .word jmpaddr

xindjump: .word $0000
          .word jmpaddr

.zeropage

value16: .word $0000
