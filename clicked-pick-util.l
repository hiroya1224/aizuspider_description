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
(defun ik-for-target-pos (target-pos &key (arm :rarm) (offset) (world nil))
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
  (send* *robot* *pre-manipulation-pose*)
  (send *robot* arm :inverse-kinematics
        (make-coords :pos (v+ offset target-pos)
                     :rpy (list 0 pi/2 0))
        :rotation-axis :z)
  )
;;;
(defun click-and-pick-it (&key (reset-pose nil) (arm :rarm) (move-time 1400))
  (when reset-pose
    ;;(send *ri* :stop-grasp)
    (send *robot* :reset-pose)
    (send *ri* :angle-vector
          (send* *robot* *pre-manipulation-pose*) move-time)
    (send *ri* :wait-interpolation)
    )

  ;; get world position which was clicked at the image
  (setq pos (get-clicked-position))

  (when (y-or-n-p (format nil "Can I pick it? ~A" pos))
    ;; inverse kinematics
    (ik-for-target-pos pos :arm arm :world nil :offset *grasp-offset*)

    ;; grasp
    (send *ri* :start-grasp arm)

    ;; pick-up
    (when (send *robot* arm :move-end-pos #f(0 0 100) :world)
      (send *ri* :angle-vector (send *robot* :angle-vector) move-time))

    ;; hold pose
    (send* *robot* *hold-object-pose*)
    (send *ri* :angle-vector (send *robot* :angle-vector) move-time)
    (send *ri* :wait-interpolation)
    (return-from click-and-pick-it t)
    )
  nil
  )
