# Atención objectos con grafos.

### Resumen de la práctica.
En esta práctica se nos pedía implementar un sistema de visión de objetos en ROS2 utilizando la herramienta de grafos y suscribiéndonos al topic de /get_model_states de gazbo para obtener las posiciones de todos los modelos dentro del mundo y atender sólo las que estuvieran a menos de 5 metros de distancia.


## Navegación.
Lo primero que teníamos que hacer era que el robot se moviera por todo el mapa, para ir detectando los distintos objetos de la zona, para implementar esta funcionalidad lo que hemos hecho ha sido hacer uso de prácticas anteriores, en este caso la de planificación.
Nuestra navegación se basa en un plan “infinito” que consiste en varios goals de posiciones concretas. Una vez llega a la última posición vuelve a pasar al primer plan.
```
(:durative-action move_location
    :parameters (?robot - robot ?prev_room - location ?next_room - location)
    :duration (= ?duration 5)
    :condition 
      (and
        (at start(robotat ?robot ?prev_room))
      )
    :effect 
      (and  
        (at end(robotat ?robot ?next_room))
        (at start(not(robotat ?robot ?prev_room)))
      )
)
```
Además hemos tenido que modificar ciertos parametros de la navegación para que el robot se moviera más rapido y la toleracia del goal fuera mucho menos restrictiva.


## Atención.
Esta ha sido la parte mas difícil, pues para empezar teníamos que suscribirnos al topic de “/get_model_states” del propio gazebo. Para hacer esto hemos necesitado instalar un plugin dentro del entorno del simulador para poder obtener el topic, pues de por si no existía. Este topic lo que hace es decirte la posición de todos los objetos que están dentro de nuestra simulación.
![WhatsApp Image 2022-05-09 at 11 37 26 PM](https://user-images.githubusercontent.com/73531592/167507471-444275cd-b48b-4332-85a3-339fdcbe8574.jpeg)

Una vez instalado el topic debíamos guardar todas estas posiciones de las tfs de estos objetos dentro de un grafo. En este grafo guardamos las posiciones de las tfs que están dentro del rango de 5 metros o en caso de estar a mayor distancia, una tf vacia.

![WhatsApp Image 2022-05-09 at 11 36 10 PM](https://user-images.githubusercontent.com/73531592/167507362-60f453ad-22e2-462a-b6a9-70ac85cc4719.jpeg)

![WhatsApp Image 2022-05-09 at 11 37 26 PM](https://user-images.githubusercontent.com/73531592/167507397-15ec6c63-e596-4e45-a8cf-d1db78f032b5.jpeg)

Una vez obtenido el grafo simplemente nos quedaba, sacar la información del grafo y con esta información mover el joint de la cabeza del robot para que observara al objeto en cuestión.
Para mover la cabeza del robot como todas las tfs tenían altura 0, lo único que hemos tenido que hacer ha sido movernos hacia los lados. Para obtener la rotación de la cabeza con respecto al punto deseado ( el objeto al que atender), hemos conseguido las coordenadas del modelo y del robot gracias a las tfs y hemos realizado un atan2, para conseguir el ángulo deseado.
Con esto el robot ya podría girar la cabeza hasta el modelo deseado y mantener la vista unos 5 segundos.

![WhatsApp Image 2022-05-09 at 11 37 26 PM](https://user-images.githubusercontent.com/73531592/167507605-0a358fa0-f106-435f-aa6d-79a38634405e.jpeg)

