extends Node3D

# Only the observer camera renders this layer, never the simulated sensor.
const LAYER = 1 << 19
const COLORS = [Color("ff665c"), Color("67e67e"), Color("6daaff")]
const CORNER_NAMES = ["TL", "TR", "BR", "BL"]

var model: RefCounted
var enabled = false
var reference_frames: Dictionary = {"O": true, "G": false, "C": false, "M": false}
var readout = ""
var frames: Dictionary = {}
var markers: Array[Node3D] = []
var lines: ImmediateMesh
var line_material: StandardMaterial3D


static func to_engine(v: Vector3) -> Vector3:
	return Vector3(-v.y, v.z, -v.x)


func _material(color: Color) -> StandardMaterial3D:
	var result = StandardMaterial3D.new()
	result.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	result.albedo_color = color
	result.no_depth_test = true
	return result


func _mesh(parent: Node3D, mesh: Mesh, mat: Material) -> MeshInstance3D:
	var instance = MeshInstance3D.new()
	instance.mesh = mesh
	instance.material_override = mat
	instance.layers = LAYER
	instance.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	parent.add_child(instance)
	return instance


func _label(parent: Node3D, text: String, at: Vector3, color: Color) -> void:
	var label = Label3D.new()
	label.text = text
	label.position = at
	label.font_size = 20
	label.pixel_size = 0.0006
	label.fixed_size = true
	label.modulate = color
	label.billboard = BaseMaterial3D.BILLBOARD_ENABLED
	label.no_depth_test = true
	label.layers = LAYER
	parent.add_child(label)


func _build() -> void:
	line_material = _material(Color.WHITE)
	line_material.vertex_color_use_as_albedo = true
	lines = ImmediateMesh.new()
	_mesh(self, lines, line_material)
	for spec in [["O", 0.8], ["G", 0.4], ["C", 0.22], ["M", 0.16]]:
		var frame = Node3D.new()
		add_child(frame)
		frames[spec[0]] = frame
		var mesh = ImmediateMesh.new()
		_mesh(frame, mesh, line_material)
		mesh.surface_begin(Mesh.PRIMITIVE_LINES)
		for axis in range(3):
			var end = Vector3.ZERO
			end[axis] = spec[1]
			mesh.surface_set_color(COLORS[axis])
			mesh.surface_add_vertex(Vector3.ZERO)
			mesh.surface_add_vertex(end)
			_label(frame, "%s.%s" % [spec[0], "XYZ"[axis]], end * 1.15, COLORS[axis])
		mesh.surface_end()
	var sphere = SphereMesh.new()
	sphere.radius = 0.009
	sphere.height = 0.018
	var corner_material = _material(Color("ffdd68"))
	for index in range(16):
		var marker = Node3D.new()
		add_child(marker)
		_mesh(marker, sphere, corner_material)
		_label(marker, "%d" % [index % 4], Vector3(0, 0.025, 0), Color("ffdd68"))
		markers.append(marker)


func _frame(key: String, origin: Vector3, x: Vector3, y: Vector3, z: Vector3) -> void:
	frames[key].transform = Transform3D(Basis(to_engine(x), to_engine(y), to_engine(z)), to_engine(origin))


func update_coordinates(observer_view: bool) -> void:
	visible = enabled and observer_view
	if not visible:
		return
	if frames.is_empty():
		_build()
	var origin: Vector3 = model.settings.gimbal.origin
	var camera: Dictionary = model.camera_pose()
	var f: Vector3 = camera.forward
	var left: Vector3 = -camera.right
	var up: Vector3 = camera.up
	_frame("O", origin, Vector3.RIGHT, Vector3.UP, Vector3.BACK)
	_frame("G", origin, f, left, up)
	_frame("C", camera.origin, camera.right, -up, f)
	var muzzle: Vector3 = origin + f * model.settings.gimbal.barrel_length
	_frame("M", muzzle, f, left, up)
	var shown = PackedStringArray()
	for key in reference_frames:
		frames[key].visible = reference_frames[key]
		if reference_frames[key]:
			shown.append(key)
	var frame_names = " / ".join(shown) if not shown.is_empty() else "无（仅显示角点）"
	readout = "X 红 / Y 绿 / Z 蓝 · 单位 m\n参考系 %s\n相机 O %s\n枪口 O %s\n四角数值始终使用 odom：\n" % [frame_names, _point(camera.origin - origin), _point(muzzle - origin)]
	for marker in markers:
		marker.visible = false
	lines.clear_surfaces()
	var observed: Array = model.camera_observation().armors
	if observed.is_empty():
		readout += "当前没有可见装甲"
		return
	lines.surface_begin(Mesh.PRIMITIVE_LINES)
	lines.surface_set_color(Color("ffdd68"))
	for observation in observed:
		var index: int = observation.id
		# The same scoring rectangle and ordering as the ROS armors message.
		var corners: Array[Vector3] = model.armor_corners(index, model.time)
		readout += "可见装甲 %d\n" % observed.find(observation)
		for corner in range(4):
			var marker = markers[index * 4 + corner]
			marker.position = to_engine(corners[corner])
			marker.visible = true
			lines.surface_add_vertex(marker.position)
			lines.surface_add_vertex(to_engine(corners[(corner + 1) % 4]))
			readout += "  %d %s %s\n" % [corner, CORNER_NAMES[corner], _point(corners[corner] - origin)]
	lines.surface_end()


func _point(point: Vector3) -> String:
	return "(%.3f, %.3f, %.3f)" % [point.x, point.y, point.z]
