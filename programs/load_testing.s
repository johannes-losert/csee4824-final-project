addi x1, x0, 2
addi x2, x0, 2
sw x2, 256(x0)
lw x5, 256(x0)
mul x7,	x2,	x1 #
add x3, x2, x7
wfi


