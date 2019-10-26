(define (domain srp-md)
  (:requirements
    :typing
  )

  (:types
    surface
    end_effector
  )

  (:predicates
    (on ?top_obj - object ?bot_obj - object)
    (at ?obj - object ?surf - surface)
    (clear ?obj - object)
    (free ?grip - end_effector)
    (held ?obj - object ?grip - end_effector)
  )

  (:action pick_from_stack
    :parameters (?obj - object ?from - object ?grip - end_effector)
    :precondition (and (clear ?obj) (on ?obj ?from) (free ?grip))
    :effect (and (clear ?from) (held ?obj ?grip) (not (free ?grip)) (not (on ?obj ?from)))
  )

  (:action pick_from_surface
    :parameters (?obj - object ?from - surface ?grip - end_effector)
    :precondition (and (clear ?obj) (at ?obj ?from) (free ?grip))
    :effect (and (held ?obj ?grip) (not (free ?grip)) (not (at ?obj ?from)))
  )

  (:action place_on_stack
    :parameters (?obj - object ?to - object ?grip - end_effector)
    :precondition (and (clear ?to) (held ?obj ?grip))
    :effect (and (free ?grip) (on ?obj ?to) (not (held ?obj ?grip)) (not (clear ?to)))
  )

  (:action place_on_surface
    :parameters (?obj - object ?to - surface ?grip - end_effector)
    :precondition (and (held ?obj ?grip))
    :effect (and (free ?grip) (at ?obj ?to) (not (held ?obj ?grip)))
  )
)
