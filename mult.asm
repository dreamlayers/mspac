;
;  Multiplication, with 0 == 0.0 and 0xFFFF == 1.0
;  Copyright (C) 2011 Boris Gjenero
;
;  This file is part of MSPRGB.
;
;  MSPRGB is free software: you can redistribute it and/or modify
;  it under the terms of the GNU General Public License as published by
;  the Free Software Foundation, either version 3 of the License, or
;  (at your option) any later version.
;
;  MSPRGB is distributed in the hope that it will be useful,
;  but WITHOUT ANY WARRANTY; without even the implied warranty of
;  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;  GNU General Public License for more details.
;
;  You should have received a copy of the GNU General Public License
;  along with MSPRGB.  If not, see <http://www.gnu.org/licenses/>.
;

        NAME mult

        EXTERN rgbout
        EXTERN rgbval

        PUBLIC mult
;        PUBLIC square
        RSEG CODE

; unsigned short square(unsigned short a)
; r12 = a, r12 = result
; clobbers: r14, r13
;square;
;        mov.w   r12,r13         ; [1] b = a
; unsigned short mult(unsigned short a, unsigned short b)
; r12 = a, r13 = b, r12 = result
; clobbers: r14 = 0, r13 = (r13 & 1) + (r13 >> 1)
mult;
        mov.w   r12,r14         ; [1] Separate result and a

        xor.w   r12,r12         ; [1] Zero result, clear carry

        rrc.w   r13             ; [1] b = b / 2
        addc.w  #0, r13         ; [1] Round up b

        setc                    ; Bit signals end of a
multlp;
        rrc.w   r14             ; [1]
        jnc     multzb          ;
        jz      multend         ; Bit input via setc above just shifted out

        clrc
        rrc.w   r12             ; [1]
        add.w   r13,r12         ; [1] 1 bit in a, so add b
        jmp     multlp

multzb;
        rrc.w   r12             ; [1] Carry was already clear
        clrc
        jmp     multlp

multend;
        ret
        END
