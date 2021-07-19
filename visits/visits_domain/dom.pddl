(define (domain localization)

    (:requirements
        :typing
        :durative-actions
        :numeric-fluents
        :negative-preconditions
        :action-costs
        :conditional-effects
        :equality
        :fluents
    )

    (:types
        robot
        region
    )

    (:predicates
    	(robot_in ?v - robot ?r - region)
        (visited ?r - region )
        (last_path ?from ?to - region)
    )

    (:functions
        (canGo)
    	(act-cost)
        (triggered ?from ?to - region)
        (dummy)
    )

    (:durative-action goto_region
    	:parameters (?v - robot ?from ?to - region)
    	:duration (= ?duration 100)
    	:condition (and (at start (robot_in ?v ?from)) (at start (< (canGo) 1)))
    	:effect (and (at start (not (robot_in ?v ?from))) (at end (visited ?to))
                     (at end (robot_in ?v ?to)) (at end (last_path ?from ?to))
                     (at end (increase (canGo) 1))
                )
    )

    (:durative-action compute_cost
        :parameters (?v - robot ?from ?to - region)
        :duration (= ?duration 0.001)
        :condition (and (at start (last_path ?from ?to)) (at start (> (canGo) 0)))
        :effect (and (at start (increase (triggered ?from ?to) 1))
                     (at end (not (last_path ?from ?to)))
                     (at end (assign (triggered ?from ?to) 0))
                     (at end (increase (act-cost) (dummy)))
                     (at end (assign (canGo) 0))
                )
    )
)
