# Attention objects with graphs.

In this practice we were asked to implement an object vision system in ROS2 using the graph tool and subscribing to the /get_model_states topic of gazbo to obtain the positions of all the models in the world and attend only those that were less than 5 meters away.

### Navigation.
The first thing we had to do was to make the robot move around the map, to detect the different objects in the area, to implement this functionality what we have done was to make use of previous practices, in this case the planning.
Our navigation is based on an "infinite" plan consisting of several goals of specific positions. Once it reaches the last position it goes back to the first plan.
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
In addition we had to modify certain parameters of the navigation so that the robot moved faster and the goal tolerance was much less restrictive.

## Attention.
This has been the most difficult part, because to start with we had to subscribe to the "/get_model_states" topic of the gazebo itself. To do this we needed to install a plugin within the simulator environment to get the topic, because it did not exist. This topic what it does is to tell you the position of all the objects that are inside our simulation. 

![WhatsApp Image 2022-05-09 at 11 36 12 PM](https://user-images.githubusercontent.com/73531592/167507706-a49950b0-d766-407e-8571-20861923c202.jpeg)

Once the topic was installed we had to save all these positions of the tfs of these objects in a graph. In this graph we save the positions of the tfs that are within the range of 5 meters or in case of being at a greater distance, an empty tf.

![WhatsApp Image 2022-05-09 at 11 36 10 PM](https://user-images.githubusercontent.com/73531592/167507362-60f453ad-22e2-462a-b6a9-70ac85cc4719.jpeg)

![WhatsApp Image 2022-05-09 at 11 37 26 PM](https://user-images.githubusercontent.com/73531592/167507397-15ec6c63-e596-4e45-a8cf-d1db78f032b5.jpeg)

Once the graph was obtained, we simply had to extract the information from the graph and with this information move the joint of the robot's head to observe the object in question.
To move the head of the robot as all the tfs had height 0, the only thing we had to do was to move sideways. To obtain the rotation of the head with respect to the desired point (the object to be observed), we have obtained the coordinates of the model and the robot thanks to the tfs and we have made an atan2, to obtain the desired angle.
With this the robot could now turn its head to the desired model and keep the view for about 5 seconds.

![WhatsApp Image 2022-05-09 at 11 36 11 PM](https://user-images.githubusercontent.com/73531592/167507720-30ad07df-fdd2-4779-ae5d-bfd6db0e3937.jpeg)



###                                                                     ---- Español ----

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

![WhatsApp Image 2022-05-09 at 11 36 12 PM](https://user-images.githubusercontent.com/73531592/167507706-a49950b0-d766-407e-8571-20861923c202.jpeg)

Una vez instalado el topic debíamos guardar todas estas posiciones de las tfs de estos objetos dentro de un grafo. En este grafo guardamos las posiciones de las tfs que están dentro del rango de 5 metros o en caso de estar a mayor distancia, una tf vacia.

![WhatsApp Image 2022-05-09 at 11 36 10 PM](https://user-images.githubusercontent.com/73531592/167507362-60f453ad-22e2-462a-b6a9-70ac85cc4719.jpeg)

![WhatsApp Image 2022-05-09 at 11 37 26 PM](https://user-images.githubusercontent.com/73531592/167507397-15ec6c63-e596-4e45-a8cf-d1db78f032b5.jpeg)

Una vez obtenido el grafo simplemente nos quedaba, sacar la información del grafo y con esta información mover el joint de la cabeza del robot para que observara al objeto en cuestión.
Para mover la cabeza del robot como todas las tfs tenían altura 0, lo único que hemos tenido que hacer ha sido movernos hacia los lados. Para obtener la rotación de la cabeza con respecto al punto deseado ( el objeto al que atender), hemos conseguido las coordenadas del modelo y del robot gracias a las tfs y hemos realizado un atan2, para conseguir el ángulo deseado.
Con esto el robot ya podría girar la cabeza hasta el modelo deseado y mantener la vista unos 5 segundos.

![WhatsApp Image 2022-05-09 at 11 36 11 PM](https://user-images.githubusercontent.com/73531592/167507720-30ad07df-fdd2-4779-ae5d-bfd6db0e3937.jpeg)
