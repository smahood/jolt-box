(ns jolt-box.box2d.ffi
  "Raw bindings to the Box2D shim in native/jb2_shim.c: a thin defcfn layer
  with no logic. See native/jb2_shim.h for the ABI each binding mirrors.
  World ids are :uint32, body and shape ids :uint64, and 0 means failure.
  Functions returning :int return 0 (ok) or -1 (bad id)."
  (:require [jolt.ffi :as ffi]))

;; --- worlds -------------------------------------------------------------------
(ffi/defcfn world-create   "jb2_world_create"   [:double :double :double] :uint32)
(ffi/defcfn world-destroy  "jb2_world_destroy"  [:uint32] :int)
(ffi/defcfn world-valid    "jb2_world_is_valid" [:uint32] :int)
(ffi/defcfn world-step     "jb2_world_step"     [:uint32 :double :int] :int)

;; --- bodies -------------------------------------------------------------------
(ffi/defcfn body-create    "jb2_body_create"
  [:uint32 :int :double :double :double :int :int :uint64] :uint64)
(ffi/defcfn body-destroy   "jb2_body_destroy"   [:uint64] :int)
(ffi/defcfn body-valid     "jb2_body_is_valid"  [:uint64] :int)
(ffi/defcfn body-read      "jb2_body_read"      [:uint64 :pointer] :int)
(ffi/defcfn body-mass-info "jb2_body_mass_info" [:uint64 :pointer] :int)
(ffi/defcfn body-user-data "jb2_body_user_data" [:uint64] :uint64)

;; --- shapes -------------------------------------------------------------------
(ffi/defcfn body-add-box   "jb2_body_add_box"
  [:uint64 :double :double :double :double :double :double :double :double :int] :uint64)
(def SHAPE-CONTACT-EVENTS 1)   ; JB2_SHAPE_* flag bits
(def SHAPE-HIT-EVENTS 2)
(def SHAPE-SENSOR 4)
(def SHAPE-SENSOR-EVENTS 8)
(ffi/defcfn body-add-circle "jb2_body_add_circle"
  [:uint64 :double :double :double :double :double :double :int] :uint64)
(ffi/defcfn body-add-capsule "jb2_body_add_capsule"
  [:uint64 :double :double :double :double :double :double :double :double :int] :uint64)
(ffi/defcfn shape-valid    "jb2_shape_is_valid" [:uint64] :int)
(ffi/defcfn shape-body     "jb2_shape_body"     [:uint64] :uint64)
(ffi/defcfn shape-set-friction    "jb2_shape_set_friction"    [:uint64 :double] :int)
(ffi/defcfn shape-set-restitution "jb2_shape_set_restitution" [:uint64 :double] :int)
(ffi/defcfn shape-set-density     "jb2_shape_set_density"     [:uint64 :double] :int)
(ffi/defcfn shape-enable-contact-events "jb2_shape_enable_contact_events" [:uint64 :int] :int)
(ffi/defcfn shape-enable-hit-events     "jb2_shape_enable_hit_events"     [:uint64 :int] :int)
(ffi/defcfn shape-enable-sensor-events  "jb2_shape_enable_sensor_events"  [:uint64 :int] :int)
(ffi/defcfn shape-set-filter "jb2_shape_set_filter" [:uint64 :uint64 :uint64 :int] :int)
(def MAX-OVERLAPS 64)       ; JB2_MAX_OVERLAPS
(ffi/defcfn shape-sensor-overlaps "jb2_shape_sensor_overlaps" [:uint64 :pointer :int] :int)
(ffi/defcfn body-mass-data "jb2_body_mass_data" [:uint64 :pointer] :int)

;; --- body motion --------------------------------------------------------------
(ffi/defcfn body-apply-impulse "jb2_body_apply_impulse"
  [:uint64 :double :double :int :double :double :int] :int)
(ffi/defcfn body-apply-force "jb2_body_apply_force"
  [:uint64 :double :double :int :double :double :int] :int)
(ffi/defcfn body-set-velocity  "jb2_body_set_velocity"  [:uint64 :double :double] :int)
(ffi/defcfn body-set-angular-velocity "jb2_body_set_angular_velocity" [:uint64 :double] :int)
(ffi/defcfn body-set-transform "jb2_body_set_transform" [:uint64 :double :double :double] :int)
(ffi/defcfn body-set-awake     "jb2_body_set_awake"     [:uint64 :int] :int)
(ffi/defcfn body-set-gravity-scale "jb2_body_set_gravity_scale" [:uint64 :double] :int)
(ffi/defcfn body-set-damping   "jb2_body_set_damping"   [:uint64 :double :double] :int)

;; --- contacts -----------------------------------------------------------------
(def MAX-CONTACTS 64)       ; JB2_MAX_CONTACTS
(def CONTACT-DOUBLES 6)     ; JB2_CONTACT_DOUBLES
(ffi/defcfn body-contacts "jb2_body_contacts" [:uint64 :pointer :pointer :int] :int)

;; --- contact events -----------------------------------------------------------
(def EVENTS-BEGIN 0)        ; JB2_EVENTS_*
(def EVENTS-END 1)
(def EVENTS-HIT 2)
(def EVENT-IDS 4)           ; JB2_EVENT_IDS
(def EVENT-DOUBLES 5)       ; JB2_EVENT_DOUBLES
(ffi/defcfn world-contact-events "jb2_world_contact_events"
  [:uint32 :int :pointer :pointer :int] :int)
(ffi/defcfn world-sensor-events "jb2_world_sensor_events" [:uint32 :int :pointer :int] :int)

;; --- joints -------------------------------------------------------------------
(ffi/defcfn joint-valid    "jb2_joint_is_valid"  [:uint64] :int)
(ffi/defcfn joint-destroy  "jb2_joint_destroy"   [:uint64] :int)
(ffi/defcfn joint-wake     "jb2_joint_wake"      [:uint64] :int)
(ffi/defcfn joint-set-collide-connected "jb2_joint_set_collide_connected" [:uint64 :int] :int)
(ffi/defcfn joint-bodies   "jb2_joint_bodies"    [:uint64 :pointer] :int)
(def JOINT-DOUBLES 11)      ; JB2_JOINT_DOUBLES
(ffi/defcfn joint-read     "jb2_joint_read"      [:uint64 :pointer] :int)

;; selectors for the <type>-set functions (JB2_SET_*)
(def SET-SPRING-ON 0)
(def SET-SPRING-HERTZ 1)
(def SET-SPRING-DAMPING 2)
(def SET-TARGET 3)
(def SET-LIMIT-ON 4)
(def SET-LIMITS 5)
(def SET-MOTOR-ON 6)
(def SET-MOTOR-SPEED 7)
(def SET-MAX-MOTOR 8)
(def SET-LENGTH 9)

(def PRISMATIC-PARAMS 18)   ; JB2_PRISMATIC_PARAMS
(def PRISMATIC-READ 15)     ; JB2_PRISMATIC_READ
(ffi/defcfn prismatic-create "jb2_prismatic_create" [:uint32 :uint64 :uint64 :pointer :uint64] :uint64)
(ffi/defcfn prismatic-set    "jb2_prismatic_set"    [:uint64 :int :double :double] :int)
(ffi/defcfn prismatic-read   "jb2_prismatic_read"   [:uint64 :pointer] :int)

(def REVOLUTE-PARAMS 16)    ; JB2_REVOLUTE_PARAMS
(def REVOLUTE-READ 12)      ; JB2_REVOLUTE_READ
(ffi/defcfn revolute-create "jb2_revolute_create" [:uint32 :uint64 :uint64 :pointer :uint64] :uint64)
(ffi/defcfn revolute-set    "jb2_revolute_set"    [:uint64 :int :double :double] :int)
(ffi/defcfn revolute-read   "jb2_revolute_read"   [:uint64 :pointer] :int)

(def DISTANCE-PARAMS 15)    ; JB2_DISTANCE_PARAMS
(def DISTANCE-READ 12)      ; JB2_DISTANCE_READ
(ffi/defcfn distance-create "jb2_distance_create" [:uint32 :uint64 :uint64 :pointer :uint64] :uint64)
(ffi/defcfn distance-set    "jb2_distance_set"    [:uint64 :int :double :double] :int)
(ffi/defcfn distance-read   "jb2_distance_read"   [:uint64 :pointer] :int)
