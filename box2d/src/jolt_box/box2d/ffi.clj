(ns jolt-box.box2d.ffi
  "Raw bindings to the Box2D shim in native/jb2_shim.c: a thin defcfn layer
  with no logic. See native/jb2_shim.h for the ABI each binding mirrors.
  World ids are :uint32, body and shape ids :uint64, and 0 means failure.
  Functions returning :int return 0 (ok) or -1 (bad id)."
  (:require [jolt.ffi :as ffi]))

;; --- worlds -------------------------------------------------------------------
(ffi/defcfn world-create   "jb2_world_create"   [:double :double] :uint32)
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
  [:uint64 :double :double :double :double :double :double :double :double] :uint64)
