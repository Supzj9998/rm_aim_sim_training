extends SceneTree

const WorldView = preload("res://scripts/world_view.gd")
const Overlay = preload("res://scripts/coordinate_overlay.gd")
var failures: Array[String] = []
var checks = 0


func _initialize() -> void:
	call_deferred("_run")


func check(condition: bool, message: String) -> void:
	checks += 1
	if not condition:
		failures.append(message)
		push_error(message)


func rendered() -> void:
	for _frame in range(3):
		await process_frame
		await RenderingServer.frame_post_draw


func check_layers(node: Node) -> void:
	if node is VisualInstance3D:
		check(node.layers == Overlay.LAYER, "every overlay visual is restricted to the observer layer")
	for child in node.get_children():
		check_layers(child)


func _run() -> void:
	var main = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	await process_frame
	if not main.has_graphics:
		push_error("Coordinate overlay test needs a graphical display")
		quit(1)
		return
	main.model.paused = true
	main.set_physics_process(false)
	main.set_process_input(false)
	var overlay: Node3D = main.world.coordinate_overlay
	check(not overlay.enabled and not overlay.visible, "coordinates are off by default")
	check(main.world.first_person, "default view stays first person")
	main._toggle_view()
	await rendered()
	var raw_before: PackedByteArray = main.world.raw_sensor_viewport.get_texture().get_image().get_data()
	var sensor_before: PackedByteArray = main.world.sensor_viewport.get_texture().get_image().get_data()
	main._toggle_view()
	main.coordinate_toggle.button_pressed = true
	await rendered()
	check(not main.world.first_person and not root.disable_3d, "enabling coordinates switches to observer view")
	check(overlay.visible and main.coordinate_panel.visible, "coordinate overlay and panel are visible")
	check(not main.first_person_image.visible, "observer view is not covered by first person texture")
	check(main.world.sensor_camera.cull_mask & Overlay.LAYER == 0, "sensor excludes coordinate render layer")
	check(main.world.main_camera.cull_mask & Overlay.LAYER != 0, "observer includes coordinate render layer")
	check_layers(overlay)
	check(raw_before == main.world.raw_sensor_viewport.get_texture().get_image().get_data(),
		"coordinates leave every raw sensor pixel unchanged")
	check(sensor_before == main.world.sensor_viewport.get_texture().get_image().get_data(),
		"coordinates leave every processed sensor pixel unchanged")
	for key in ["O", "G", "C", "M"]:
		main.coordinate_frames[key].button_pressed = true
	await rendered()
	for key in ["O", "G", "C", "M"]:
		check(overlay.frames[key].visible, "all four reference frames can be shown together")
	check(raw_before == main.world.raw_sensor_viewport.get_texture().get_image().get_data()
		and sensor_before == main.world.sensor_viewport.get_texture().get_image().get_data(),
		"showing all reference frames leaves both sensor images unchanged")
	var observed: Array = main.model.camera_observation().armors
	check(not observed.is_empty(), "default pose has visible armor")
	check(main.coordinate_readout.text.contains("可见装甲 0"), "panel identifies current visible array index")
	for observation in observed:
		var index: int = observation.id
		var corners: Array[Vector3] = main.model.armor_corners(index, main.model.time)
		for corner in range(4):
			var marker: Node3D = overlay.markers[index * 4 + corner]
			check(marker.visible and marker.position.is_equal_approx(WorldView.to_engine(corners[corner])),
				"visible markers use the exact ROS scoring corners in TL/TR/BR/BL order")
	main.model.yaw = 0.34
	main.model.pitch = -0.21
	main.world.update_world()
	var origin: Vector3 = main.model.settings.gimbal.origin
	var camera: Dictionary = main.model.camera_pose()
	check(overlay.frames.O.position.is_equal_approx(WorldView.to_engine(origin)), "odom origin is the gimbal pivot")
	check(overlay.frames.O.basis.x.is_equal_approx(WorldView.to_engine(Vector3.RIGHT)), "odom axes stay fixed after aiming")
	check(overlay.frames.G.basis.x.is_equal_approx(WorldView.to_engine(camera.forward)), "gimbal X follows actual aim")
	check(overlay.frames.C.position.is_equal_approx(WorldView.to_engine(camera.origin)), "optical origin follows camera offset")
	check(overlay.frames.C.basis.x.is_equal_approx(WorldView.to_engine(camera.right))
		and overlay.frames.C.basis.y.is_equal_approx(WorldView.to_engine(-camera.up))
		and overlay.frames.C.basis.z.is_equal_approx(WorldView.to_engine(camera.forward)), "optical axes point right, down, forward")
	check(overlay.frames.M.position.is_equal_approx(WorldView.to_engine(origin + camera.forward * main.model.settings.gimbal.barrel_length)),
		"muzzle origin follows barrel length and actual aim")
	main.model.yaw = 0.0
	main.model.pitch = 0.0
	main.model.rotation_origin = PI * 0.75
	main.world.update_world()
	main._update_status()
	await rendered()
	check(main.model.camera_observation().armors.size() == 2, "diagonal target exposes two plates")
	check(main.coordinate_readout.text.contains("可见装甲 1"), "panel shows all visible plates")
	main.coordinate_frames.G.button_pressed = false
	for key in ["O", "G", "C", "M"]:
		check(overlay.frames[key].visible == (key != "G"), "unchecking one frame preserves the others")
	for key in ["O", "G", "C", "M"]:
		main.coordinate_frames[key].button_pressed = false
		check(not overlay.frames[key].visible, "every reference frame can be unchecked independently")
	check(main.coordinate_readout.text.contains("仅显示角点"), "panel explains an empty reference selection")
	for observation in main.model.camera_observation().armors:
		for corner in range(4):
			check(overlay.markers[observation.id * 4 + corner].visible, "corners remain visible without reference axes")
	main.coordinate_frames.O.button_pressed = true
	main.coordinate_frames.G.button_pressed = true
	check(overlay.frames.O.visible and overlay.frames.G.visible and not overlay.frames.C.visible,
		"odom and gimbal frames can be selected together")
	main.coordinate_toggle.button_pressed = false
	main.coordinate_toggle.button_pressed = true
	check(overlay.frames.O.visible and overlay.frames.G.visible and not overlay.frames.C.visible,
		"master toggle preserves the multiple frame selection")
	await rendered()
	root.get_texture().get_image().save_png("/tmp/rm_aim_coordinate_overlay.png")
	main._toggle_view()
	await rendered()
	check(not overlay.visible and not main.coordinate_panel.visible, "first person hides coordinate graphics and panel")
	main._toggle_view()
	await rendered()
	check(overlay.visible and main.coordinate_panel.visible, "observer view restores enabled coordinates")
	check(overlay.frames.O.visible and overlay.frames.G.visible and not overlay.frames.C.visible,
		"view changes preserve the multiple frame selection")
	main.model.health = 0
	main.world.update_world()
	check(overlay.readout.contains("当前没有可见装甲"), "no observation shows an empty result")
	for marker in overlay.markers:
		check(not marker.visible, "old corner markers disappear when no plates are observed")
	main.coordinate_toggle.button_pressed = false
	check(not overlay.visible and not main.coordinate_panel.visible, "switch disables all coordinate graphics")
	print("PASS COORDINATES: %d checks" % checks if failures.is_empty() else "FAILED COORDINATES: " + str(failures))
	main.queue_free()
	await process_frame
	quit(0 if failures.is_empty() else 1)
