(require :aizuspider "package://aizuspider_description/AizuSpiderSS.l")
(require :robot-interface "package://pr2eus/robot-interface.l")

(defclass aizuspider-interface
  ;;:super robot-move-base-interface
  :super robot-interface
  :slots ()
  )
(defmethod aizuspider-interface
  (:init
   (&rest args)
   (let ((r (instance AizuSpiderSS-robot :init)))
     ;; make controller configuration from joint-list
     #|
     (dolist (limb (list 'rarm 'lleg 'rleg))
       (let ((key (intern (string limb) *keyword-package*))
             (mname (intern (string-upcase (format nil "~A-controller" limb)) *keyword-package*))
             )
         ;;
         (eval
          `(defmethod aizuspider-interface
             (,mname
              nil
              (list
               (list
                (cons :controller-action ,(format nil "~A_controller/follow_joint_trajectory" limb) )
                (cons :controller-state  ,(format nil "~A_controller/state" limb) )
                (cons :action-type control_msgs::FollowJointTrajectoryAction)
                (cons :joint-names
                      ',(mapcar #'(lambda (j) (send j :name)) (send r key :joint-list)) )
                )
               )))
          )
         ))
     |#
     ;;
     (send-super*
      :init
      :robot r
      ;; :namespace "AizuSpider"
      ;; :move-base-action-name "move_base"
      ;; :base-frame-id "/base_link"
      ;; :odom-topic "/odom"
      :groupname "aizuspider_interface" args)
     )
   ;;
   #|
   (dolist (ct (list :rarm-controller
                     :rleg-controller
                     :lleg-controller))
     (send self :add-controller ct :create-actions nil))
   |#
   )
  ;;
  #|
  (:default-controller nil
   (append (send self :rarm-controller)
           (send self :rleg-controller)
           (send self :lleg-controller))
   )
  |#
  (:default-controller nil
   (list
    (list
     (cons :controller-action "fullbody_controller/follow_joint_trajectory")
     (cons :controller-state  "fullbody_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "FR_FLIPPER" "FL_FLIPPER" "BR_FLIPPER" "BL_FLIPPER"
                              "SHOULDER" "ARM" "FOREARM" "WRIST1" "WRIST2" "HAND"
                              "FINGER1" "FINGER2" "FINGER3"
                              ))
     ))
   )
  #|
  (:rarm-controller nil
   (list
    (list
     (cons :controller-action "rarm_controller/follow_joint_trajectory")
     (cons :controller-state  "rarm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "r_shoulder_p_joint" "r_shoulder_r_joint" "r_shoulder_y_joint"
                              "r_elbow_joint" "r_wrist_y_joint" "r_wrist_p_joint" "r_wrist_r_joint"
                              "r_hand_y_joint"
                              ))
     ))
   )
  (:larm-controller nil
   (list
    (list
     (cons :controller-action "larm_controller/follow_joint_trajectory")
     (cons :controller-state  "larm_controller/state")
     (cons :action-type control_msgs::FollowJointTrajectoryAction)
     (cons :joint-names (list "l_shoulder_p_joint" "l_shoulder_r_joint" "l_shoulder_y_joint"
                              "l_elbow_joint" "l_wrist_y_joint" "l_wrist_p_joint" "l_wrist_r_joint"
                              "l_hand_y_joint"
                              ))
     ))
   )
  |#
  )

(defun aizuspider-init (&rest args &key (namespace "AizuSpider") &allow-other-keys)
  (if (not (boundp '*ri*))
      (setq *ri* (instance* aizuspider-interface :init
                            :namespace namespace args)))
  (if (not (boundp '*robot*))
      (setq *robot* (AizuSpiderSS)))
  )
