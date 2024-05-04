.section .text
.align 4
    li t1, 1
    li t2, 2
    bne t1,t2, bt1
    addi x1, x0, 1
    addi x2, x0, 2
    addi x3, x0, 3
    addi x4, x0, 4
    mul x7,	x2,	x1 #
    add x3, x2, x7
    wfi


bt1:
    addi x1, x0, 5
    addi x2, x0, 6
    addi x3, x0, 7
    addi x4, x0, 8

end:
    mul x7,	x2,	x1 #
    add x3, x2, x7
    wfi

