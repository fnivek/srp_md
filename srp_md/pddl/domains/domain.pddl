(define (domain srp-md)
  (:requirements
    :typing
  )

  (:types
    block table - object
  )

  (:predicates
    (on ?x - object ?y - object)
    (clear ?x - object)
  )

  (:action move
    :parameters (?x - object ?to - object ?from - object)
    :precondition (and (clear ?x) (clear ?to) (on ?x ?from))
    :effect (and (clear ?from) (on ?x ?to) (not (clear ?to)) (not (on ?x ?from)))
  )
)
