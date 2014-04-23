# -*- encoding: utf-8 -*-
import sys
import os
import datetime

from pilasengine import escenas
from pilasengine import imagenes
from pilasengine import actores
from pilasengine import utils
from pilasengine import fondos
from pilasengine import depurador

import widget

VERSION = "0.86"


class Pilas(object):
    """Representa el area de juego de pilas, el componente principal.

    El objeto pilas se inicializa cuando llamamos a la función
    ``pilasengine.iniciar()``. El objeto que se retorna es un
    objeto de esta clase.

    Internamente, este objeto será el que representa la ventana
    principal. Es es contenedor de la escena, el punto de contrucción
    de los actores y quien mantiene con "vida" el juego completo.
    """

    def __init__(self, ancho=640, alto=480, titulo='pilas-engine', con_aceleracion=True, habilitar_mensajes_log=False):
        self.habilitar_mensajes_log(habilitar_mensajes_log)
        self.log("Iniciando pilas con una ventana de ", ancho, "x", alto)
        self.actores = actores.Actores(self)
        self.escenas = escenas.Escenas(self)
        self.imagenes = imagenes.Imagenes(self)
        self.utils = utils.Utils(self)
        self.fondos = fondos.Fondos(self)
        self.depurador = depurador.Depurador(self)
        self.escenas.Normal()

        if con_aceleracion:
            self.widget = widget.WidgetConAceleracion(self, ancho, alto)
        else:
            self.widget = widget.WidgetSinAceleracion(self, ancho, alto)

    def usa_aceleracion(self):
        """Informa si está habilitado el modo aceleración de video."""
        return (self.widget.__class__ == widget.WidgetConAceleracion)

    def obtener_widget(self):
        return self.widget

    def obtener_centro_fisico(self):
        """Retorna el centro de la ventana en pixels."""
        return self.widget.obtener_centro_fisico()

    def obtener_coordenada_de_pantalla_absoluta(self, x, y):
        dx, dy = self.widget.obtener_centro_fisico()
        return (x + dx, dy - y)

    def obtener_area(self):
        """Retorna el tamaño real de la ventana."""
        return self.widget.obtener_area()

    def habilitar_mensajes_log(self, estado):
        self._imprimir_mensajes_log = estado

    def obtener_escena_actual(self):
        return self.escenas.obtener_escena_actual()

    def realizar_actualizacion_logica(self):
        self.escenas.realizar_actualizacion_logica()

    def realizar_dibujado(self, painter):
        try:
            self.escenas.realizar_dibujado(painter)
            self.depurador.realizar_dibujado(painter)
        except Exception, e:
            # Si hay un error lo informa y detiene toda
            # la ejecución de pilas.
            self.log("Capturando un error: %s", e)
            self.depurador.desactivar_todos_los_modos()
            error = self.escenas.Error(e)
            raise e

    def log(self, *mensaje):
        "Muestra un mensaje de prueba sobre la consola."

        if self._imprimir_mensajes_log:
            hora = datetime.datetime.now().strftime("%H:%M:%S")
            mensaje = map(lambda x: str(x), mensaje)
            print(":: %s :: %s " %(hora, " ".join(mensaje)))

    def obtener_ruta_al_recurso(self, ruta):
        """Busca la ruta a un archivo de recursos.

        Los archivos de recursos (como las imagenes) se buscan en varios
        directorios (ver docstring de image.load), así que esta
        función intentará dar con el archivo en cuestión.

        :param ruta: Ruta al archivo (recurso) a inspeccionar.
        """
        self.log("Buscando ruta al recurso:", ruta)
        return utils.obtener_ruta_al_recurso(ruta)


def iniciar(ancho=640, alto=480, titulo='Pilas'):
    """
    Inicia la ventana principal del juego con algunos detalles de funcionamiento.

    Ejemplo de invocación:

        >>> pilas.iniciar(ancho=320, alto=240)

    .. image:: images/iniciar_320_240.png

    Parámetros:

    :ancho: el tamaño en pixels para la ventana.
    :alto: el tamaño en pixels para la ventana.
    :titulo: el titulo a mostrar en la ventana.
    """
    pilas = Pilas(ancho=ancho, alto=alto, titulo=titulo)
    return pilas

def abrir_asistente():
    import asistente
    return asistente.abrir()

def abrir_manual():
    import manual
    return manual.abrir()

def abrir_interprete():
    import interprete
    return interprete.abrir()
