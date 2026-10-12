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
(ffi/defcfn shape-valid    "jb2_shape_is_valid" [:uint64] :int)
(ffi/defcfn shape-body     "jb2_shape_body"     [:uint64] :uint64)
(ffi/defcfn shape-set-friction    "jb2_shape_set_friction"    [:uint64 :double] :int)
(ffi/defcfn shape-set-restitution "jb2_shape_set_restitution" [:uint64 :double] :int)
(ffi/defcfn shape-set-density     "jb2_shape_set_density"     [:uint64 :double] :int)
(ffi/defcfn shape-enable-contact-events "jb2_shape_enable_contact_events" [:uint64 :int] :int)
(ffi/defcfn shape-enable-hit-events     "jb2_shape_enable_hit_events"     [:uint64 :int] :int)
(ffi/defcfn shape-enable-sensor-events  "jb2_shape_enable_sensor_events"  [:uint64 :int] :int)
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
