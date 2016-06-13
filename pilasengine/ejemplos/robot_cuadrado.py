# -*- encoding: utf-8 -*-
import pilasengine


def cuadrado(robot):
    robot.forward(50, 2)
    robot.turnLeft(60, 1)

pilas = pilasengine.iniciar()
b = pilas.actores.Board("/dev/tty/USB0")
r = pilas.actores.Robot(b, 1)

pilas.avisar("El Robot hace un cuadrado.")

for i in range(4):
    cuadrado(r)

pilas.ejecutar()
