(define (domain nei)
(:requirements :strips :typing :negative-preconditions :adl :fluents :durative-actions)
(:types
  room - location
  door elevator - connector
  robot object tool - item
)
(:predicates 
  (robotat ?r - robot ?l - location)
)

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

)

