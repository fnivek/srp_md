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
    (above ?top_obj - object ?bot_obj - object)
    (at ?obj - object ?surf - surface)
    (clear ?obj - object)
    (free ?grip - end_effector)
    (held ?obj - object ?grip - end_effector)
    (proximity ?obj - object ?base_obj - object)
  )

  (:action pick_from_surface
    :parameters (?obj - object ?from - surface ?grip - end_effector)
    :precondition (and (clear ?obj) (at ?obj ?from) (free ?grip))
    :effect (and (held ?obj ?grip) (not (free ?grip)) (not (at ?obj ?from)))
  )

  (:action place_on_surface
    :parameters (?obj - object ?to - surface ?grip - end_effector)
    :precondition (and (held ?obj ?grip))
    :effect (and (free ?grip) (at ?obj ?to) (not (held ?obj ?grip)))
  )

  (:action place_on_stack1
    :parameters (?obj - object ?to - object ?surf - surface ?grip - end_effector)
    :precondition (and (clear ?to) (held ?obj ?grip) (at ?to ?surf))
    :effect (and (free ?grip) (on ?obj ?to) (not (held ?obj ?grip)) (not (clear ?to)) (above ?obj ?to))
  )

  (:action pick_from_stack1
    :parameters (?obj - object ?from - object ?surf - surface ?grip - end_effector)
    :precondition (and (clear ?obj) (on ?obj ?from) (free ?grip) (at ?from ?surf))
    :effect (and (clear ?from) (held ?obj ?grip) (not (free ?grip)) (not (on ?obj ?from)) (not (above ?obj ?from)))
  )

  (:action place_on_stack2
    :parameters (?obj - object ?to - object ?surf - surface ?grip - end_effector ?stack2 - object)
    :precondition (and (clear ?to) (held ?obj ?grip) (on ?to ?stack2) (at ?stack2 ?surf))
    :effect (and (free ?grip) (on ?obj ?to) (above ?obj ?to) (not (held ?obj ?grip)) (not (clear ?to)) (above ?obj ?stack2))
  )

  (:action pick_from_stack2
    :parameters (?obj - object ?from - object ?surf - surface ?grip - end_effector ?stack2 - object)
    :precondition (and (clear ?obj) (on ?obj ?from) (free ?grip) (on ?from ?stack2) (at ?stack2 ?surf))
    :effect (and (clear ?from) (held ?obj ?grip) (not (free ?grip)) (not (on ?obj ?from)) (not (above ?obj ?from)) (not (above ?obj ?stack2)))
  )

  (:action place_on_stack3
    :parameters (?obj - object ?to - object ?surf - surface ?grip - end_effector ?stack2 - object ?stack3 - object)
    :precondition (and (clear ?to) (held ?obj ?grip) (on ?to ?stack2) (on ?stack2 ?stack3) (at ?stack3 ?surf))
    :effect (and (free ?grip) (on ?obj ?to) (above ?obj ?to) (not (held ?obj ?grip)) (not (clear ?to)) (above ?obj ?stack2) (above ?obj ?stack3))
  )

  (:action pick_from_stack3
    :parameters (?obj - object ?from - object ?surf - surface ?grip - end_effector ?stack2 - object ?stack3 - object)
    :precondition (and (clear ?obj) (on ?obj ?from) (free ?grip) (on ?from ?stack2) (on ?stack2 ?stack3) (at ?stack3 ?surf))
    :effect (and (clear ?from) (held ?obj ?grip) (not (free ?grip)) (not (on ?obj ?from)) (not (above ?obj ?from)) (not (above ?obj ?stack2)) (not (above ?obj ?stack3)))
  )

  (:action place_on_stack4
    :parameters (?obj - object ?to - object ?surf - surface ?grip - end_effector ?stack2 - object ?stack3 - object ?stack4 - object)
    :precondition (and (clear ?to) (held ?obj ?grip) (on ?to ?stack2) (on ?stack2 ?stack3) (on ?stack3 ?stack4) (at ?stack4 ?surf))
    :effect (and (free ?grip) (on ?obj ?to) (above ?obj ?to) (not (held ?obj ?grip)) (not (clear ?to)) (above ?obj ?stack2) (above ?obj ?stack3) (above ?obj ?stack4))
  )

  (:action pick_from_stack4
    :parameters (?obj - object ?from - object ?surf - surface ?grip - end_effector ?stack2 - object ?stack3 - object ?stack4 - object)
    :precondition (and (clear ?obj) (on ?obj ?from) (free ?grip) (on ?from ?stack2) (on ?stack2 ?stack3) (on ?stack3 ?stack4) (at ?stack4 ?surf))
    :effect (and (clear ?from) (held ?obj ?grip) (not (free ?grip)) (not (on ?obj ?from)) (not (above ?obj ?from)) (not (above ?obj ?stack2)) (not (above ?obj ?stack3)) (not (above ?obj ?stack4)))
  )

  ; Commented out to save computational time
  ; (:action place_on_stack5
  ;   :parameters (?obj - object ?to - object ?surf - surface ?grip - end_effector ?stack2 - object ?stack3 - object ?stack4 - object ?stack5 - object)
  ;   :precondition (and (clear ?to) (held ?obj ?grip) (on ?to ?stack2) (on ?stack2 ?stack3) (on ?stack3 ?stack4) (on ?stack4 ?stack5) (at ?stack5 ?surf))
  ;   :effect (and (free ?grip) (on ?obj ?to) (above ?obj ?to) (not (held ?obj ?grip)) (not (clear ?to)) (above ?obj ?stack2) (above ?obj ?stack3) (above ?obj ?stack4) (above ?obj ?stack5))
  ; )

  ; (:action pick_from_stack5
  ;   :parameters (?obj - object ?from - object ?surf - surface ?grip - end_effector ?stack2 - object ?stack3 - object ?stack4 - object ?stack5 - object)
  ;   :precondition (and (clear ?obj) (on ?obj ?from) (free ?grip) (on ?from ?stack2) (on ?stack2 ?stack3) (on ?stack3 ?stack4) (on ?stack4 ?stack5) (at ?stack5 ?surf))
  ;   :effect (and (clear ?from) (held ?obj ?grip) (not (free ?grip)) (not (on ?obj ?from)) (not (above ?obj ?from)) (not (above ?obj ?stack2)) (not (above ?obj ?stack3)) (not (above ?obj ?stack4)) (not (above ?obj ?stack5)))
  ; )

  (:action place_on_surf_proximity_to
    :parameters (?obj - object ?base_obj - object ?surf - surface ?grip - end_effector)
    :precondition (and (held ?obj ?grip))
    :effect (and (free ?grip) (at ?obj ?surf) (not (held ?obj ?grip)) (proximity ?obj ?base_obj))
  )
)
