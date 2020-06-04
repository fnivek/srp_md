(define (domain srp-md)
  (:requirements
    :typing
    :adl
    :equality
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
    (proximity ?obj - object ?other_obj - object)
    (in_box ?obj - object)
  )

  (:action pick_from_surface
    :parameters (?obj - object ?from - surface ?grip - end_effector)
    :precondition (and (clear ?obj) (at ?obj ?from) (free ?grip))
    :effect (and (held ?obj ?grip) (not (free ?grip)) (not (at ?obj ?from)) (not (in_box ?obj))
              (forall (?prox_obj)
                (when (or (proximity ?obj ?prox_obj) (proximity ?prox_obj ?obj))
                  (and (not (proximity ?obj ?prox_obj)) (not (proximity ?prox_obj ?obj))))))
  )

  (:action place_on_surface
    :parameters (?obj - object ?to - surface ?grip - end_effector)
    :precondition (and (held ?obj ?grip))
    :effect (and (free ?grip) (at ?obj ?to) (not (held ?obj ?grip)) (in_box ?obj))
  )

  (:action place_on_stack
    :parameters (?obj - object ?to - object ?grip - end_effector)
    :precondition (and (clear ?to) (held ?obj ?grip) (in_box ?to))
    :effect (and (free ?grip) (on ?obj ?to) (not (held ?obj ?grip)) (not (clear ?to)) (in_box ?obj))
  )

  (:action pick_from_stack
    :parameters (?obj - object ?from - object ?grip - end_effector)
    :precondition (and (clear ?obj) (on ?obj ?from) (free ?grip))
    :effect (and (clear ?from) (held ?obj ?grip) (not (free ?grip)) (not (on ?obj ?from)) (not (in_box ?obj))
              (forall (?prox_obj)
                (when (or (proximity ?obj ?prox_obj) (proximity ?prox_obj ?obj))
                  (and (not (proximity ?obj ?prox_obj)) (not (proximity ?prox_obj ?obj))))))
  )

  (:action place_on_surf_proximity_to
    :parameters (?obj - object ?other_obj - object ?surf - surface ?grip - end_effector)
    :precondition (and (held ?obj ?grip) (in_box ?other_obj))
    :effect (and (free ?grip) (at ?obj ?surf) (not (held ?obj ?grip)) (proximity ?obj ?other_obj)
             (proximity ?other_obj ?obj) (in_box ?obj)
             (forall (?prox_obj)
              (when (or (proximity ?other_obj ?prox_obj) (proximity ?prox_obj ?other_obj))
                (and (proximity ?obj ?prox_obj) (proximity ?prox_obj ?obj)))))
  )

  (:action place_on_obj_proximity_to
    :parameters (?obj - object ?other_obj - object ?bot_obj - object ?grip - end_effector)
    :precondition (and (held ?obj ?grip) (in_box ?other_obj) (in_box ?bot_obj) (not (= ?other_obj ?bot_obj)))
    :effect (and (free ?grip) (on ?obj ?bot_obj) (not (held ?obj ?grip)) (proximity ?obj ?other_obj)
             (proximity ?other_obj ?obj) (in_box ?obj)
             (forall (?prox_obj)
              (when (or (proximity ?other_obj ?prox_obj) (proximity ?prox_obj ?other_obj))
                (and (proximity ?obj ?prox_obj) (proximity ?prox_obj ?obj)))))
  )
)
