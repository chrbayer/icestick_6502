; ---------------------------------------------------------------------------
; test_asm.s
; ---------------------------------------------------------------------------

.export         _test_asm

.segment  "CODE"

.proc _test_asm: near
        lda     #$12
        ldx     #$34
        ldy     #$56
        ldz     #$78
        map
        eom
        dew     value16
        inw     value16
        neg
        neg
        dec     value32
        neg
        neg
        inc     value32
        clc
        neg
        neg
        ror     value32
        neg
        neg
        rol     value32
        neg
        neg
        neg
        neg
        neg
        neg
        lda     #$00
        tax
        tay
        taz
        neg
        neg
        bit     value32
        lda     #$08
        neg
        neg
        bit     value32
        neg
        neg
        lda     value32
        neg
        neg
        rol
        neg
        neg
        ror
        neg
        neg
        inc
        neg
        neg
        dec
        neg
        neg
        adc     value32
        neg
        neg
        sta     value32
        neg
        neg
        adc     value32
        neg
        neg
        sta     value32
        neg
        neg
        eor     value32
        lda     #<value32
        sta     indptr
        lda     #>value32
        sta     indptr+1
        clc
        neg
        neg
        lda     (indptr),Z
        neg
        neg
        adc     (indptr),Z
        neg
        neg
        sta     (indptr),Z
        neg
        neg
        adc     (indptr),Z
        neg
        neg
        sta     (indptr),Z
        neg
        neg
        eor     (indptr),Z
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

value32: .dword $43272376

.zeropage

value16: .word $4327

indptr:  .word $0000
