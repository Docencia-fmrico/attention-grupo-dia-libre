(define (domain nei)
(:requirements :strips :typing :negative-preconditions :adl :fluents :durative-actions)
(:types
  room - location
  door elevator - connector
  robot object tool - item
)
(:predicates 
  (robotat ?r - robot ?l - location)

  (objectat ?o - object ?l - location)
  (robottool ?r - robot ?t - tool)
  (toolfree ?t - tool)
  (robotcarrying ?r - robot ?t - tool ?o - object)
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

(:durative-action pick
  :parameters (?o - object ?l - location ?r - robot ?t - tool)
  :duration (= ?duration 5)
  :condition 
    (and
      (at start(robottool ?r ?t))
      (at start(objectat ?o ?l))
      (at start(robotat ?r ?l))
      (at start(toolfree ?t))
    )
:effect 
  (and 
    (at end(robotcarrying ?r ?t ?o))
    (at start(not (objectat ?o ?l)))
    (at start(not (toolfree ?t)))
  )
)

(:durative-action drop
:parameters (?o - object ?l - location ?r - robot ?t - tool)
:duration (= ?duration 5)
:condition 
  (and 
    (at start(robottool ?r ?t))
    (at start(robotat ?r ?l))
    (at start(robotcarrying ?r ?t ?o))
  )
:effect 
  (and 
    (at end(objectat ?o ?l))
    (at end(toolfree ?t))
    (at start(not (robotcarrying ?r ?t ?o)))
  )
)



)

