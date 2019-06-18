(require :jaxon_jvrc-interface "package://hrpsys_choreonoid_tutorials/euslisp/jaxon_jvrc-interface.l")

(require :clicked-pick-util "package://aizuspider_description/clicked-pick-utils.l")

(defun initialize-clicked-pick ()
  (jaxon_jvrc-init)
  (setq *robot* (jaxon_jvrc))
  ;;; fixed variables
  (setq *grasp-offset*   (float-vector -15 0 0))

  (setq *pre-manipulation-pose* (list :reset-manip-pose))
  (setq *hold-object-pose*      (list :reset-manip-pose))

  (setq *fixed-frame-id* "WAIST")
  )

(warn "
(initialize-clicked-pick)
")
