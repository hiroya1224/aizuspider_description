(require :aizuspider-interface "package://aizuspider_description/aizuspider-interface.l")

(require :clicked-pick-util "package://aizuspider_description/clicked-pick-utils.l")

(defun initialize-clicked-pick ()
  (aizuspider-init)
  (setq *robot* (aizuspider))

  (setq *grasp-offset*   (float-vector -15 0 0))

  (setq *pre-manipulation-pose* (list :reset-manip-pose))
  (setq *hold-object-pose*      (list :reset-pose))

  (setq *fixed-frame-id* "AizuSpiderAA/CHASSIS")
  )

(warn "
(initialize-clicked-pick)
")
