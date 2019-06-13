(require :aizuspider-interface "package://aizuspider_description/aizuspider-interface.l")
;;;
(defun convert-frame-of-point (msg &key (frame-id *fixed-frame-id*))
  ;;(setq *msg-geo* msg)
  (let* ((hd (send msg :header))
         (pt (ros::tf-point->pos (send msg :point)))
         (tm (send hd :stamp))
         (fm (send hd :frame_id))
         pos
         )
    (when (send *tfl* :wait-for-transform frame-id fm tm 5)
      (let* ((cds (send *tfl* :lookup-transform frame-id fm tm))
             )
        (setq pos (send cds :transform-vector pt))
        ))
    pos
    ))
;;;
(defun get-clicked-position ()
  (warn ";;; Please click target point on the image view~%")
  (let((pos
        (convert-frame-of-point
         (one-shot-subscribe
          "/pointcloud_screenpoint_nodelet/output_point"
          geometry_msgs::pointstamped))))
    (warn ";;; get point ~A~%" pos)
    pos))
;;;
;;;
(defun ik-for-target-pos (target-pos &key (offset) (world nil))
  (unless offset (setq offset (float-vector 0 0 0)))
  (when world
    (let ((cur-base
           (send *tfl* :lookup-transform "map" "base_footprint" (ros::time 0))))
      (setq target-pos
            (send
             (send cur-base :transformation (make-coords :pos target-pos))
             :worldpos))
      ))
  (send *robot* :reset-coords)
  (send *robot* :reset-manip-pose)
  (send *robot* :inverse-kinematics
        (make-coords :pos (v+ offset target-pos)
                     :rpy (list 0 pi/2 0))
        :rotation-axis :x)

  )
;;;
(defun click-and-pick-it (&key (reset-pose nil))
  (when reset-pose
    ;;(send *ri* :stop-grasp)
    (send *robot* :reset-pose)
    (send *ri* :angle-vector  (send *robot* :reset-pose) 1200)
    (send *ri* :wait-interpolation)
    )

  (setq pos (get-clicked-position))

  (when (y-or-n-p (format nil "Can I pick it? ~A" pos))
    (ik-for-target-pos pos :world nil :offset *grasp-offset*)
    ;;(move-as-robot-model)
    ;;(send *ri* :start-grasp)
    ;;(unix::usleep (* 500 1000));; sleep
    ;;;
    ;;;(grasp-with-torso)

    (send *robot* :reset-manip-pose)
    (send *ri* :angle-vector (send *robot* :angle-vector) 1200)
    (send *ri* :wait-interpolation)
    (return-from click-and-pick-it t)
    )
  nil
  )

(defun initialize-clicked-pick ()
  (aizuspider-init)
  (setq *robot* (aizuspider))
  ;;; fixed variables
  (setq *ungrasp-offset* (float-vector 0 0 300))
  (setq *grasp-offset*   (float-vector 0 0 15))
  (setq *fixed-frame-id* "AizuSpiderAA/CHASSIS")
  )

(warn "
(initialize-clicked-pick)
(clicked-tidy-demo)
")
