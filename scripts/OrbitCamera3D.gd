extends Camera3D
class_name OrbitCamera3D

## Cámara orbital 3D interactiva para el visor estructural de Structura 3D.
## Controles:
##   - Botón derecho o central + arrastrar: Orbitar alrededor del pivote
##   - Shift + Botón derecho/central + arrastrar: Desplazar (Pan)
##   - Rueda del ratón: Zoom acercar/alejar
##   - Tecla R: Restablecer vista isométrica

var target: Vector3 = Vector3(1.5, 0.75, 0.0)
var distance: float = 7.5
var yaw_deg: float = -35.0
var pitch_deg: float = 24.0

var min_distance: float = 0.5
var max_distance: float = 150.0
var orbit_sensitivity: float = 0.35
var pan_sensitivity: float = 0.0025
var zoom_factor: float = 0.9

var _is_orbiting: bool = false
var _is_panning: bool = false


func _ready() -> void:
	update_camera_transform()


func _unhandled_input(event: InputEvent) -> void:
	if event is InputEventMouseButton:
		var mb := event as InputEventMouseButton
		if mb.button_index == MOUSE_BUTTON_RIGHT or mb.button_index == MOUSE_BUTTON_MIDDLE:
			if mb.pressed:
				if Input.is_key_pressed(KEY_SHIFT):
					_is_panning = true
					_is_orbiting = false
				else:
					_is_orbiting = true
					_is_panning = false
			else:
				_is_orbiting = false
				_is_panning = false
		elif mb.button_index == MOUSE_BUTTON_WHEEL_UP and mb.pressed:
			distance = clampf(distance * zoom_factor, min_distance, max_distance)
			update_camera_transform()
		elif mb.button_index == MOUSE_BUTTON_WHEEL_DOWN and mb.pressed:
			distance = clampf(distance / zoom_factor, min_distance, max_distance)
			update_camera_transform()

	elif event is InputEventMouseMotion:
		var mm := event as InputEventMouseMotion
		if _is_orbiting:
			yaw_deg -= mm.relative.x * orbit_sensitivity
			pitch_deg = clampf(pitch_deg + mm.relative.y * orbit_sensitivity, -85.0, 85.0)
			update_camera_transform()
		elif _is_panning:
			var right_vec := global_transform.basis.x
			var up_vec := global_transform.basis.y
			target += (-right_vec * mm.relative.x + up_vec * mm.relative.y) * distance * pan_sensitivity
			update_camera_transform()

	elif event is InputEventKey:
		var ke := event as InputEventKey
		if ke.pressed and ke.keycode == KEY_R:
			reset_view()


func reset_view(center: Vector3 = Vector3(1.5, 0.75, 0.0), dist: float = 7.5) -> void:
	target = center
	distance = dist
	yaw_deg = -35.0
	pitch_deg = 24.0
	update_camera_transform()


func update_camera_transform() -> void:
	var yaw_rad := deg_to_rad(yaw_deg)
	var pitch_rad := deg_to_rad(pitch_deg)
	var offset := Vector3(
		distance * cos(pitch_rad) * sin(yaw_rad),
		distance * sin(pitch_rad),
		distance * cos(pitch_rad) * cos(yaw_rad)
	)
	global_position = target + offset
	look_at(target, Vector3.UP)
